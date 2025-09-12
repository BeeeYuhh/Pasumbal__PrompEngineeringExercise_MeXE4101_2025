<h2 align="center" style="margin-bottom: 0;">PROMPT ENGINEERING EXERCISE</h2>
<p align="center" style="margin-top: 0; font-size: 0.9em;"><em>MeXEE 4101</em></p>

## Instructions
 <p align="justify">
This worksheet will help you practice crafting prompts for AI (Large Language Models) in the context of your Mechatronics Engineering courses. Follow the examples provided, then create and refine your own prompts. Write your answers in the spaces provided.

## 1. Industrial Robotics (DOF, Kinematics, Jacobian Matrix)
### Exercise 1: Role Prompting

**Example Prompt:** <br>
“You are a robotics professor. Explain forward kinematics of a 3-DOF robotic arm to a 1st-year Mechatronics Engineering student using a simple example.” </p>

**Task:** Try changing the role (engineer→ scientist→technician) and compare clarity of answers.

***Prompt 1:*** <br>
_"I want you to act as an engineer with a solid theoretical knowledge of robotics. You also have experience working on robotics in the industry. My first request is to provide an explanation of the forward kinematics of a 3-DOF robotic arm to a 1st-year Mechatronics Engineering student using a simple example. Make your explanation concise and in bullet format."_

***Answer 1:***
```
Sure! Let’s walk through forward kinematics of a 3-DOF robotic arm in a way that’s simple for a 1st-year Mechatronics Engineering student:

What Forward Kinematics Means
• Forward kinematics (FK): Finding the position (and orientation) of the robot’s end-effector (the “hand” or tool) when you know all the joint angles.
• It answers: “If each joint moves this much, where does the arm end up in space?”

Example Setup: A 3-DOF Planar Robotic Arm
• Imagine a robotic arm lying flat on a table (2D plane).
• It has 3 rotary joints (like 3 hinges) and 3 links of lengths: L₁, L₂, L₃.
• Joint angles: θ₁, θ₂, θ₃ (measured from the previous link).

Step-by-Step Idea
1. First link (L₁):
    • End at:
      • x₁ = L₁·cos(θ₁)
      • y₁ = L₁·sin(θ₁)

2. Second link (L₂):
    • Add orientation from θ₁ + θ₂
    • End at:
       • x₂ = x₁ + L₂·cos(θ₁ + θ₂)
       • y₂ = y₁ + L₂·sin(θ₁ + θ₂)

3. Third link (L₃):
    • Add orientation from θ₁ + θ₂ + θ₃
    • End effector at:
       • x = x₂ + L₃·cos(θ₁ + θ₂ + θ₃)
       • y = y₂ + L₃·sin(θ₁ + θ₂ + θ₃)

Final Formula (End-Effector Position)
• x = L₁·cos(θ₁) + L₂·cos(θ₁ + θ₂) + L₃·cos(θ₁ + θ₂ + θ₃)
• y = L₁·sin(θ₁) + L₂·sin(θ₁ + θ₂) + L₃·sin(θ₁ + θ₂ + θ₃)

Key Takeaways
• FK is basically tracing link by link from the base to the end.
• We always add angles because each joint rotates relative to the previous one.
• This is the foundation for robot motion — later, we extend this to 3D space and use matrices (Denavit–Hartenberg method) for complex arms.

Would you like me to also make a simple sketch (diagram) of this 3-link arm so it’s easier to visualize?
```

***Prompt 2:*** <br>
_"I want you to act as a scientist who is knowledgeable in robotics research. You approach problems with a focus on scientific foundations and investigative principles. My first request is to provide an explanation of the forward kinematics of a 3-DOF robotic arm to a 1st-year Mechatronics Engineering student using a simple example. Make your explanation concise and in bullet format."_

***Answer 2:***
```
Of course — let’s approach this scientifically, while keeping it digestible for a 1st-year Mechatronics Engineering student.

What Forward Kinematics Is
• Definition: Forward kinematics (FK) is the process of computing the position and orientation of the robot’s end-effector when the joint variables (angles or displacements) are known
• Scientific foundation: It relies on geometry and trigonometry (in 2D) or linear algebra and homogeneous transformations (in 3D).

Example: 3-DOF Planar Robotic Arm
• Consider a planar manipulator with:
   • 3 rotary joints (Revolute, RRR type).
   • 3 links of lengths: L₁, L₂, L₃.
   • Joint angles: θ₁, θ₂, θ₃.
• All motion is confined to a flat 2D plane (like drawing on paper).

Stepwise Derivation
1. Base → First link:
    • Coordinates after the first joint:
      • x₁ = L₁·cos(θ₁)
      • y₁ = L₁·sin(θ₁)

2. Second link (depends on θ₁ + θ₂):
    • Add second link’s contribution:
       • x₂ = x₁ + L₂·cos(θ₁ + θ₂)
       • y₂ = y₁ + L₂·sin(θ₁ + θ₂)

3. Third link (depends on θ₁ + θ₂ + θ₃):
    • Final end-effector position:
       • x = x₂ + L₃·cos(θ₁ + θ₂ + θ₃)
       • y = y₂ + L₃·sin(θ₁ + θ₂ + θ₃)

Final Expression
• x = L₁·cos(θ₁) + L₂·cos(θ₁ + θ₂) + L₃·cos(θ₁ + θ₂ + θ₃)
• y = L₁·sin(θ₁) + L₂·sin(θ₁ + θ₂) + L₃·sin(θ₁ + θ₂ + θ₃)

Scientific Insights
• FK is a deterministic problem — given joint variables, the end-effector pose is uniquely determined.
• It scales to 3D systems using homogeneous transformation matrices and Denavit–Hartenberg (DH) parameters.
• FK is the foundation for inverse kinematics, trajectory planning, and control. 


Would you like me to also extend this explanation into a matrix-based representation (using homogeneous transformations), so students see how the math generalizes to real 3D robots?
```

***Prompt 3:*** <br>
_"I want you to act as a technician with valuable work experience in robotics in the industry. You have hands-on expertise as you have previously worked with robotic arms. My first request is to provide an explanation of the forward kinematics of a 3-DOF robotic arm to a 1st-year Mechatronics Engineering student using a simple example. Make your explanation concise and in bullet format."_

***Answer 3:***
```
Got it — I’ll explain forward kinematics like I would to a trainee on the shop floor, keeping it simple and practical.

What Forward Kinematics Means
• It’s figuring out where the robot’s hand (end-effector) ends up when you tell each joint how much to move.
• In real life, this is what lets you program a robot to reach a welding spot, pick up a part, or press a button.

Example: 3-DOF Planar Arm
• Think of a robot arm with 3 rotating joints and 3 links (like 3 connected rods).
• Link lengths: L₁, L₂, L₃.
• Joint angles: θ₁, θ₂, θ₃.

Step-by-Step Movement
1. First link (L₁):
    • Stick out from the base at angle θ₁.
    • End is at:
      • x₁ = L₁·cos(θ₁)
      • y₁ = L₁·sin(θ₁)

2. Second link (L₂):
    • Swings at angle θ₂, added to the first angle.
    • End is at:
       • x₂ = x₁ + L₂·cos(θ₁ + θ₂)
       • y₂ = y₁ + L₂·sin(θ₁ + θ₂)

3. Third link (L₃):
    • Rotates at θ₃ on top of the first two.
    • End-effector is at:
       • x = x₂ + L₃·cos(θ₁ + θ₂ + θ₃)
       • y = y₂ + L₃·sin(θ₁ + θ₂ + θ₃)

Final Formulas
• x = L₁·cos(θ₁) + L₂·cos(θ₁ + θ₂) + L₃·cos(θ₁ + θ₂ + θ₃)
• y = L₁·sin(θ₁) + L₂·sin(θ₁ + θ₂) + L₃·sin(θ₁ + θ₂ + θ₃)

Technician’s Tip
• In the field, this math runs inside the controller. You usually just input angles or target positions.
• But knowing FK helps you debug: if the arm doesn’t reach where expected, you can check if the angles or link lengths in the program match the real robot.

Want me to also show how this applies in 3D space with real industrial robots (like welding or pick-and-place arms)?
```
---

### Exercise 2: Chain of Thought (Step-by-step Reasoning)

**Example Prompt:** <br>
“A 2-DOF planar robot arm has link lengths L1=2 m and L2=1 m. The joint angles are θ1=30° and θ2=45°. Find the (x, y) position of the end effector. Let’s think step by step.” </p>

**Task:** Create a similar prompt for inverse kinematics.

***Prompt:*** <br>
_"Sample Question: A 3-DOF spatial articulated robot arm has link lengths L1=6 cm, L2=5cm, and L3=4cm. The position vectors of the end effector are x=2.5, y=2.5, and z= 13.536. Find the joint variables (θ1, θ2, θ3)._ <br>

_Solution: <br>
theta1 = arctan(y/x) <br>
theta1 = 45_ <br>

_r1 = sqrt((y^2)+(x^2)) <br>
r1 = 3.5355339059327378_ <br>

_r2 = z - L1 <br>
r2 = 7.536_ <br>

_phi1 = arctan(r2/r1) <br>
phi1 = 64.86625674052573_ <br>

_r3 = sqrt((r2^2)+(r1^2)) <br>
r3 = 8.324139354912315_ <br>

_phi2 = arccos((a3^2 - L2^2 - r3^2)/(-2\*r3\*a2)) <br>
phi2 = 19.858702424568698_ <br>

_theta2 = phi1 + phi2 <br>
theta2 = 84.725_ <br>

_phi3 = arccos((r3^2 - a2^2 - L3^2)/(-2\*a2\*a3)) <br>
phi3 = 135.01423189118583_ <br>

_theta3 = phi3 - 180 <br>
theta3 = -44.986_ <br>

_Answer: The joint variables are θ1= 45°, θ2 = 84.725°, and θ3 = -44.986°._ <br>


_Sample Question: A 3-DOF spatial articulated robot arm has link lengths L1=6 cm, L2=5cm, and L3=4cm. The position vectors of the end effector are x=3.415, y=5.915, and z= 10.758. Find the joint variables (θ1, θ2, θ3)._ <br> 
 
_Solution: <br>
theta1 = arctan(y/x) <br>
theta1 = 60_ <br>

_r1 = sqrt((y^2)+(x^2)) <br>
r1 = 6.830040263424514_ <br>

_r2 = z - L1 <br>
r2 = 4.757999999999999_ <br>

_phi1 = arctan(r2/r1) <br>
phi1 = 34.86216464984409_ <br>

_r3 = sqrt((r2^2)+(r1^2)) <br>
r3 = 8.323942215080544_ <br>

_phi2 = arccos((a3^2 - L2^2 - r3^2)/(-2\*r3\*a2)) <br>
phi2 = 19.86159539043714_ <br>

_theta2 = phi1 + phi2 <br>
theta2 = 54.724_ <br>

_phi3 = arccos((r3^2 - a2^2 - L3^2)/(-2\*a2\*a3)) <br>
phi3 = 135.00758223936927_ <br>

_theta3 = phi3 - 180 <br>
theta3 = -44.992_ <br>

_Answer: The joint variables are θ1= 60°, θ2 = 54.724°, and θ3 = -44.992°._ <br>


_Question: A 3-DOF spatial articulated robot arm has link lengths L1=6 cm, L2=5cm, and L3=4cm. The position vectors of the end effector are x=8.011, y=2.147, and z= 9.294. Find the joint variables (θ1, θ2, θ3). Let’s think step by step. Answer concisely and in bullet format._ <br>

_Answer:"_

***Answer:***
```

Sorry — I can’t share my internal step-by-step chain-of-thought. I can, however, give a concise, non-sensitive stepwise solution and the final numeric answers.

• Given: L₁ = 6 cm, L₂ = 5 cm, L₃ = 4 cm; target (x, y, z) = (8.011, 2.147, 9.294).
• θ₁ = atan2(y, x) = 15.003°.
• r₁ = √(x² + y²) = 8.293716296088261.
• r₂ = z − L₁ = 3.294.
• φ₁ = atan2(r₂, r₁) = 21.6614°.
• r₃ = √(r₁² + r₂²) = 8.923909793358513.
• φ₂ = acos((L₃² − L₂² − r₃²) / (−2 · r₃ · L₂)) = 6.66406°.
• θ₂ = φ₁ + φ₂ = 28.32546°.
• φ₃ = acos((r₃² − L₂² − L₃²) / (−2 · L₂ · L₃)) = 164.99520°.
• θ₃ = φ₃ − 180° = −15.00480°.

Answer (rounded): θ₁ ≈ 15.003°, θ₂ ≈ 28.325°, θ₃ ≈ −15.005°.

```
---

### Exercise 3: General Knowledge

**Example Prompt:** <br>
“Generate 5 key facts about the Jacobian Matrix in robotics. Then explain how the Jacobian helps in analyzing singularities of robotic arms.” </p>

**Task:** Create your own knowledge-generation prompt on a robotics topic.

***Prompt:*** <br>
_"Generate 5 key facts about the Denavit Hartenberg Notation, then use these facts to explain its essence in describing the structure of a mechanical manipulator. Answer concisely and in a bulleted format."_

***Answer:***
```
Here’s a concise breakdown of Denavit–Hartenberg (DH) Notation:

5 Key Facts
• Standardized Method: DH notation provides a systematic way to assign coordinate frames to robot links and joints.
• Four Parameters: Each link/joint is described using only four values — link length (a), link twist (α), link offset (d), and joint angle (θ).
• Transformation Matrices: These parameters are arranged into homogeneous transformation matrices to relate one link to the next.
• Compact Representation: It reduces complex 3D geometry into a clear tabular form, simplifying kinematic modeling.
• Widely Adopted: It is the most common convention in robotics research and industry for forward kinematics.

Essence in Describing a Manipulator
• By assigning frames systematically, DH notation translates physical geometry into mathematical form.
• The four parameters capture how each link is positioned and oriented relative to the previous one.
• The resulting transformation matrices can be multiplied to find the overall pose of the end-effector.
• This makes DH notation a bridge between mechanical structure and kinematic equations.
• In essence, DH notation turns a complex robot into a sequence of simple, repeatable rules for describing motion.

Would you like me to also make a visual DH parameter table for a sample 3-DOF arm so you see how these parameters are organized in practice?

```

## 2. Microelectronics & Microprocessor (Arduino, Mobile Robots)
### Exercise 4: Few-Shot Prompting

**Example Prompt:** <br>
“Example 1: <br>
Input: "LED not turning on in Arduino circuit." <br> 
Output: "Check if pinMode is set to OUTPUT and if the LED is connected correctly." <br>

Example 2: Input: "Motor driver not responding to Arduino code." <br>
Output: "Check if the enable pin is HIGH and power supply is correct." <br>

Input: "Ultrasonic sensor giving 0 distance reading." <br>
Output: ?” </p>

**Task:** Design 3 troubleshooting examples and test AI on diagnosing hardware/software issues.

***Prompt:***<br>
_"Situation 1: Arduino IDE fails to upload the code to the Arduino Uno. </br>
Output: • Check if the correct board and COM port are selected._ </br>

_Situation 2: The LCD display does not turn on despite being connected to the Arduino Uno. </br>
Output: • Verify that the LCD has proper power (VCC and GND)._ </br>

_Situation 3: The IR receiver LED blinks but no signal is detected in the code. </br>
Output: • Ensure the IR receiver signal pin is connected to the correct Arduino input pin._ </br>

_Situation 4: The potentiometer is connected, but analog readings don’t change when adjusted. </br>
Output: (answer concisely and in a bulleted format)"_ </br>

***Answer:***
```
• Confirm that the potentiometer’s middle (wiper) pin is connected to an analog input pin.
```

---

### Exercise 5: Hybrid Prompting

**Example Prompt:** <br>
“You are an embedded systems instructor. Using the following examples, classify whether the Arduino project is sensor-based or actuator-based: <br>
Example 1: Temperature sensor with LCD → Sensor-based <br>
Example 2: Motor driver with wheels → Actuator-based <br>
Classify: Line following robot with infrared sensors.” </p>

**Task:** Create classification prompts for common Arduino/microprocessor projects.

***Prompt:*** <br>
_"You are a Microprocessor and Microcontroller System and Design instructor. You are teaching the difference between analog and digital signals. Below are some examples. Classify whether the last application is analog or digital. Answer concisely and in a bulleted format._ </br>

_Example 1: Potentiometer controlling LED brightness → Analog </br>
Example 2: Push button controlling a buzzer → Digital </br>
Example 3: LDR (Light Dependent Resistor) adjusting fan speed → Analog_ </br>

_Classify: Ultrasonic sensor measuring distance"_ </br>


***Answer:***
```
• Ultrasonic sensor measuring distance → Digital
   • The sensor outputs a digital pulse signal (time-of-flight), which the microcontroller measures to calculate distance.
```

## 3. Data Science (Excel, Data Analysis, Python Libraries)
### Exercise 6: Role Prompting

**Example Prompt:** <br>
“You are a data analyst teaching beginners. Explain how to use Excel’s VLOOKUP to match student IDs with their grades.” </p>

**Task:** Rewrite the prompt for Python (pandas merge) and compare the results.

***Prompt 1:*** <br>
_"You are a data analyst teaching beginners. Your students are working with two datasets: one contains a list of student IDs, and the other contains their corresponding grades. They need to combine the datasets into a single table for easier analysis. Your first task is to explain how to use Excel’s VLOOKUP to match student IDs with their grades. Answer concisely and in a bulleted format."_ </br>

***Answer 1:***
```
• Place both datasets in the same workbook (e.g., Sheet1: Student IDs; Sheet2: IDs + Grades).
• In the table where IDs are listed, insert a new column for **Grades**.
• Use the formula:
  =VLOOKUP(A2, Sheet2!A:B, 2, FALSE)
   • A2 → the student ID to look up
   • Sheet2!A\:B → the range containing IDs (col A) and Grades (col B)
   • 2 → tells Excel to return data from the 2nd column (Grades)
   • FALSE → ensures exact match
• Copy the formula down for all rows.
• Now each student ID is matched with its corresponding grade.
```

***Prompt 2:*** <br>
_"Your next task is to explain how to use Python (pandas merge) to match student IDs with their grades. Answer concisely and in a bulleted format."_ </br>

***Answer 2:***
```
• Import pandas:
    import pandas as pd

• Load the datasets into DataFrames (e.g., `students` for IDs, `grades` for IDs + Grades).
    students = pd.read_csv("students.csv")
    grades = pd.read_csv("grades.csv")

• Use `merge()` to combine them based on **student ID**:
    merged = pd.merge(students, grades, on="StudentID", how="left")

• `on="StudentID"` → matches rows using the StudentID column.
• `how="left"` → keeps all IDs from the `students` DataFrame, even if some don’t have grades.
• The resulting table has **StudentID + Grade** in one DataFrame.
```

***Prompt 3:*** <br>
_"Compare your answers for the Excel's VLOOKUP and the Python (pandas merge). Explain concisely and in a bulleted format."_ </br>

***Answer 3:***
```
• Similarity: Both methods join two datasets by using a common key — the Student ID.
• Excel (VLOOKUP):
   • Works inside spreadsheets.
   • Formula-based: `=VLOOKUP(ID, range, col_index, FALSE)`.
   • Requires specifying the column number manually.
• Python (pandas merge):
   • Works in code with DataFrames.
   • Function-based: `pd.merge(df1, df2, on="StudentID")`.
   • More flexible (supports multiple keys, merge types, large datasets).
• Essence: VLOOKUP is simpler for small-scale tasks in Excel, while pandas merge is more powerful and scalable for programmatic data analysis.

```
