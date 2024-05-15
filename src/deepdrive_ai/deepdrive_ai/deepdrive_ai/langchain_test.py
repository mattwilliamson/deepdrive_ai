import rclpy
from llama_ros.langchain import LlamaROS
from langchain.prompts import PromptTemplate
from langchain_core.output_parsers import StrOutputParser

from langchain.memory import ChatMessageHistory
demo_ephemeral_chat_history = ChatMessageHistory()
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain_core.runnables.history import RunnableWithMessageHistory



def main():
    rclpy.init()

    # create the llama_ros llm for langchain
    llm = LlamaROS()

    # create a prompt template
    # prompt_template = "tell me a joke about {topic}"
    # prompt = PromptTemplate(
    #     input_variables=["topic"],
    #     template=prompt_template
    # )
    
    prompt = ChatPromptTemplate.from_messages(
        [
            (
                "system",
                "You are a helpful assistant. Answer all questions to the best of your ability.",
            ),
            MessagesPlaceholder(variable_name="chat_history"),
            ("human", "{input}"),
        ]
    )

    # chain = prompt | chat


    # create a chain with the llm and the prompt template
    chain = prompt | llm | StrOutputParser()

    chain_with_message_history = RunnableWithMessageHistory(
        chain,
        lambda session_id: demo_ephemeral_chat_history,
        input_messages_key="input",
        history_messages_key="chat_history",
    )

    # run the chain
    text = chain_with_message_history.invoke({"input": "what is Pindrop?"}, {"configurable": {"session_id": "unused"}},)

    print("------")
    print(text)
    print("------")

    text = chain_with_message_history.invoke(
        {"input": "What did I just ask you?"}, {"configurable": {"session_id": "unused"}}
    )

    print("------")
    print(text)
    print("------")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
