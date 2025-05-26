import time
from openai import OpenAI
import os
import ast, re
import rospy
from std_msgs.msg import String

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
    parsed_problem = clean_tag_id(parsed_problem)
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
    
    print(f"direction: {direction}")
    return direction

class GPTNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('gpt_node', anonymous=True)
        
        # Create publisher for direction
        self.direction_pub = rospy.Publisher('drone_direction', String, queue_size=10)
        
        # Create subscriber for QR codes
        rospy.Subscriber('qr_codes', String, self.qr_callback)
        
        rospy.loginfo("GPT Node initialized")
        
    def qr_callback(self, msg):
        # When QR code is received, process it and publish direction
        direction = GPTTaskPlan(msg.data)
        
        # Publish the direction
        direction_msg = String()
        direction_msg.data = direction
        self.direction_pub.publish(direction_msg)
        rospy.loginfo(f"Published direction: {direction}")

if __name__ == '__main__':
    try:
        gpt_node = GPTNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
