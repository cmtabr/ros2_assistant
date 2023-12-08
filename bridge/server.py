#! /usr/bin/env python3

# Libraries importing
import re
import subprocess
import time
from decouple import config
from langchain.chat_models import ChatOpenAI
from langchain.document_loaders import DirectoryLoader, TextLoader
from langchain.embeddings.sentence_transformer import SentenceTransformerEmbeddings
from langchain.prompts import ChatPromptTemplate
from langchain.schema.runnable import RunnablePassthrough
from langchain.text_splitter import CharacterTextSplitter
from langchain.vectorstores import chroma
import roslibpy
from twilio.rest import Client

# Utilities importing


# Environment variables definition
OPENAI_KEY = config('OPENAI_API_KEY')
ACCOUNT_SID = config('ACCOUNT_SID')
AUTH_TOKEN = config('AUTH_TOKEN')
FROM_NUMBER = config('FROM_NUMBER')
TO_NUMBER = config('TO_NUMBER')

# Functions definition
ros2_launched = False
def launch_ros2(ros2_launched):
    launch_command = "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
    try:
        if ros2_launched:
            subprocess.Popen(launch_command, shell=True)
            ros2_launched = True
    except subprocess.CalledProcessError as e:
        print(f"Error executing command: {e}")

# Class definition
class ChatBotModel(): 
    def __init__(self):
        self._model = ChatOpenAI(model="gpt-3.5-turbo", api_key=OPENAI_KEY)
        self._retriever = self.archive_loader_and_vectorizer()
        self._template = """\ 
        Rely only on the context to answer the question.
        {context} 
        With the context, the response to the question should be in the expected form literally:
        The item ordered by the user was -> item_name. 
        Item position is -> [x, y].

        Question: {question}
        """
        self._prompt = ChatPromptTemplate.from_template(self._template)
    
    def archive_loader_and_vectorizer(self):
        """ 
        This function loads txt documents from current directory 
        and vectorizes them
        """
        loader = DirectoryLoader('../', 
                                glob='**/*.txt',
                                loader_cls=TextLoader,
                                show_progress=True
                            )
        documents = loader.load()
        text_splitter = CharacterTextSplitter(chunk_size=30000, chunk_overlap=0)
        docs = text_splitter.split_documents(documents)
        embedding_function = SentenceTransformerEmbeddings(model_name="all-MiniLM-L6-v2")
        vectorstore = chroma.Chroma.from_documents(docs, embedding_function)
        retriever = vectorstore.as_retriever()
        return retriever
    
    def chat(self, text):
        chain = (
            {"context": self._retriever, "question": RunnablePassthrough()}
            | self._prompt
            | self._model
        )
        output_text = ""
        for s in chain.stream(text):
            print(s.content, end="", flush=True)
            output_text += s.content
        return output_text
    
class Twilio:
    def __init__(self):
        self.account_sid = ACCOUNT_SID
        self.auth_token = AUTH_TOKEN
        self.client = Client(self.account_sid, self.auth_token)
    
    def send_whatsapp(self, body, from_, to):
        try:
            message = self.client.messages.create(
                                            body=body,
                                            from_=from_,
                                            to=to
                                            )
            print("Mensagem enviada com sucesso!")
        except Exception as e:
            print("Erro ao enviar mensagem: ", e)

def main():
    launch_ros2(ros2_launched)
    twilio = Twilio()
    client = roslibpy.Ros(host='localhost', port=9090)
    client.run()
    talker = roslibpy.Topic(client, name='/state_machine_topic', 
                            message_type='std_msgs/String')
    chat_model = ChatBotModel()
    while client.is_connected:
        output = chat_model.chat('prego')
        twilio.send_whatsapp(body=output, from_='whatsapp:'+ FROM_NUMBER, to='whatsapp:'+ TO_NUMBER)
        talker.publish(roslibpy.Message({'data': output}))
        print('Sending message...')
        time.sleep(1)
        break
    talker.unadvertise()
    client.terminate()
    print("client disconnect")

if __name__ == "__main__":
    main()