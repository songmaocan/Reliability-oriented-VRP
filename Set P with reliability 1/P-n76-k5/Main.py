"""
Function: solve the Reliability-oriented CVRP (standard instance) by ADMM-based method
Author: Maocan Song
Version: 1.1 Read the variance file, if not exist, we generate the variance and output the file.
Date: 2021-4-29
"""
from Model import Method
import time
def main():
    start_time=time.time()
    mod=Method()
    mod.solving()
    end_time=time.time()
    spend_time=end_time-start_time
    mod.output_to_file(spend_time)
    print("CPU running time {} min".format(round(spend_time/60),3))

if __name__=="__main__":
    main()