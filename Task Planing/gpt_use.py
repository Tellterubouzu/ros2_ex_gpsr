from openai import OpenAI
import os
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))


response = client.chat.completions.create(model="gpt-4.1",
messages=[
    {"role": "system", "content": "You are a helpful assistant."},
    {"role": "user", "content": "こんにちは、API の使い方を教えてください。"}
])
print(response.choices[0].message.content)
