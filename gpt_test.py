import time
from openai import OpenAI
import os
import ast, re

organization_key = "-"
api_key = "-"

gpt_model = "gpt-4o"

def clean_tag_id(problem_text):
    """
    Tag ID: 가 포함된 부분에서 숫자 또는 텍스트만 남기고 제거
    """
    lines = problem_text.strip().splitlines()
    cleaned_lines = []
    for line in lines:
        if line.startswith("LEFT:"):
            content = re.sub(r"Tag ID:\s*", "", line[len("LEFT:"):]).strip()
            cleaned_lines.append(f"LEFT: {content}")
        elif line.startswith("RIGHT:"):
            content = re.sub(r"Tag ID:\s*", "", line[len("RIGHT:"):]).strip()
            cleaned_lines.append(f"RIGHT: {content}")
        else:
            cleaned_lines.append(line.strip())
    return "\n".join(cleaned_lines)

def GPTTaskPlan(parsed_problem):
    parsed_problem = clean_tag_id(problem_text)
    # print(parsed_problem)
    prompt = f"""
You are controlling an autonomous drone navigating through a field.
There is a gate ahead, and you must decide whether to go LEFT or RIGHT based on the following problem.

Your task:
- Carefully read the problem.
- Reason about the correct choice.
- Then respond naturally in one or two sentences, ending clearly with either "I will go LEFT." or "I will go RIGHT."

Here is the problem:

{parsed_problem}
"""

    client = OpenAI(
        api_key=os.environ.get("OPENAI_API_KEY", api_key),
        organization = organization_key
    ) 
    max_tokens = 100
    
    response = client.chat.completions.create(
        model=gpt_model,
        messages=[
            {
                "role": "system",
                "content": "Forget the previous conversation.",
            },
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                ],
            }
        ],
        max_tokens=max_tokens,
        temperature=0
    )
    result = response.choices[0].message.content
    
    # Parse direction from result
    if "I will go LEFT" in result:
        direction = "LEFT"
    elif "I will go RIGHT" in result:
        direction = "RIGHT"
    else:
        direction = "UNKNOWN"
    
    # print(f"Full response: {result}")
    print(f"direction: {direction}")
    return direction


# 예시 문제
problem_text = """
LEFT: Tag ID: 12
RIGHT: Tag ID: 17
Q: Which number is not prime?
"""

GPTTaskPlan(problem_text)