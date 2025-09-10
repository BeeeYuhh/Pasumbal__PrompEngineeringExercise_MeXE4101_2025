<h2 align="center" style="margin-bottom: 0;">PROMPT ENGINEERING EXERCISE</h2>
<p align="center" style="margin-top: 0; font-size: 0.9em;"><em>MeXEE 4101</em></p>

## Instructions
 <p align="justify">
This worksheet will help you practice crafting prompts for AI (Large Language Models) in the context of your Mechatronics Engineering courses. Follow the examples provided, then create and refine your own prompts. Write your answers in the spaces provided.

## 1. Industrial Robotics (DOF, Kinematics, Jacobian Matrix)
### Exercise 1: Role Prompting

<p align="justify">
Example Prompt:
“You are a robotics professor. Explain forward kinematics of a 3-DOF robotic arm to a 1st-year Mechatronics Engineering student using a simple example.” </p>
<p align="justify">
Task: Try changing the role (engineer→ scientist→technician) and compare clarity of answers.

Prompt 1: "I want you to act as an engineer with a solid theoretical knowledge of robotics. You also have experience working on robotics in the industry. My first request is to provide an explanation of the forward kinematics of a 3-DOF robotic arm to a 1st-year Mechatronics Engineering student using a simple example. Make your explanation concise and in bullet format."

Answer 1:
```
Sure! Let’s walk through forward kinematics of a 3-DOF robotic arm in a way that’s simple for a 1st-year Mechatronics Engineering student:

What Forward Kinematics Means

Forward kinematics (FK): Finding the position (and orientation) of the robot’s end-effector (the “hand” or tool) when you know all the joint angles.

It answers: “If each joint moves this much, where does the arm end up in space?”

Example Setup: A 3-DOF Planar Robotic Arm

Imagine a robotic arm lying flat on a table (2D plane).

It has 3 rotary joints (like 3 hinges) and 3 links of lengths: L₁, L₂, L₃.

Joint angles: θ₁, θ₂, θ₃ (measured from the previous link).

Step-by-Step Idea

First link (L₁):

End at:

x₁ = L₁·cos(θ₁)

y₁ = L₁·sin(θ₁)

Second link (L₂):

Add orientation from θ₁ + θ₂

End at:

x₂ = x₁ + L₂·cos(θ₁ + θ₂)

y₂ = y₁ + L₂·sin(θ₁ + θ₂)

Third link (L₃):

Add orientation from θ₁ + θ₂ + θ₃

End effector at:

x = x₂ + L₃·cos(θ₁ + θ₂ + θ₃)

y = y₂ + L₃·sin(θ₁ + θ₂ + θ₃)

Final Formula (End-Effector Position)

x = L₁·cos(θ₁) + L₂·cos(θ₁ + θ₂) + L₃·cos(θ₁ + θ₂ + θ₃)

y = L₁·sin(θ₁) + L₂·sin(θ₁ + θ₂) + L₃·sin(θ₁ + θ₂ + θ₃)

Key Takeaways

FK is basically tracing link by link from the base to the end.

We always add angles because each joint rotates relative to the previous one.

This is the foundation for robot motion — later, we extend this to 3D space and use matrices (Denavit–Hartenberg method) for complex arms.
```

Would you like me to also make a simple sketch (diagram) of this 3-link arm so it’s easier to visualize?
