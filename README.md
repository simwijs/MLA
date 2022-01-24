Hi Simon, 

I just sent you the code through wetransfer. Here are some more infos (from previous code transfers :) ) :

I took time to comment on most of the code (developed in C++) and you'll find Ma's instances in the folder instances.
The methods described in our paper correspond to the function : 
- Solve_TOTP for Ma's method
- Solve_Decentralized for their decentralized method 
- Solve_Greedy_Heuristic for our MLA*

For this project I used to develop the code on Clion but I compiled and tested my code in my terminal.

First step would be to check if you can compile the code (all the required libraries are in the project so it should compile). To do so, use your terminal, go into the project's folder and run make clean and then make. It should compile all the project and create an executable in the folder bin.

Then you can first try to run the existing bash by running bash bash.sh. This will run all the instance for our method corresponding to the method solve_type = 5 in the class Resolution_Method.cpp

Questions : 
(1)    How did you render the data from the output .yaml files into maps and animations?
(2)    I don’t see that the iterator variables,  and , which are passed as arguments from the bash file are used.  Did I miss something?

Answers :
1) For the .yaml outputs, I used a python code that I found online (folder enclosed). To create the animation, you have to : 
- Open the folder in the terminal
- Run : python3 ./example/visualize.py map.yaml output.yaml 
After a few seconds, the animation will start. You just have to make sure that python3 is installed on your computer 

2) It seems that the arguments   were just for some tests, I think that you can remove them from the bash.
Do not hesitate if you have more questions, 
Best regards, 
Florian 

