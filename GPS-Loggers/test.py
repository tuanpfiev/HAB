import subprocess

commandStr = "~/HAB/Prediction_Autologger/build/BallARENA-dev"
                            
try:
   subprocess.Popen(commandStr, shell=True)
except Exception as e:
    print("Exception: " + str(e.__class__))
    print(e)
    print("Error calling path prediction script. Will continue and call again later.")                            