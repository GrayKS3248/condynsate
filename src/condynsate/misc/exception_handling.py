from dataclasses import dataclass
import textwrap
import traceback
import sys
import os
os.system("") # For escape characters to work


def print_exception(e, function):
    msg = '\n'.join(textwrap.wrap(str(e), 60))
    bars = ''.join(['#']*60)
    s = ("{}{}{}\n".format(ESC.FAIL,bars,ESC.ENDC),
         "{}{}{}\n".format(ESC.FAIL,type(e).__name__,ESC.ENDC),
         "In function: ",
         "{}{}(){}\n".format(ESC.OKBLUE, function.__name__, ESC.ENDC),
         "{}{}{}\n".format(ESC.WARNING,  msg, ESC.ENDC),
         "{}{}{}".format(ESC.FAIL,bars,ESC.ENDC),)
    
    print("\n{}{}{}".format(ESC.FAIL, bars, ESC.ENDC), flush=True)
    _, _, tb = sys.exc_info()
    stack_summary = traceback.extract_tb(tb)
    msgs = traceback.format_list(stack_summary)
    print("Traceback (most recent call last):", flush=True)
    for m in msgs:
        print(m,end='',flush=True)
    print(''.join(s), flush=True)
    print("",flush=True)
    
    
@dataclass
class ESC:
    HEADER: str = '\033[95m'
    OKBLUE: str = '\033[94m'
    OKCYAN: str = '\033[96m'
    OKGREEN: str = '\033[92m'
    WARNING: str = '\033[93m'
    FAIL: str = '\033[91m'
    ENDC: str = '\033[0m'
    BOLD: str = '\033[1m'
    UNDERLINE: str = '\033[4m'
    
    def header(msg):
        self = ESC()
        return "{}{}{}".format(self.HEADER,msg,self.ENDC)
    
    def okblue(msg):
        self = ESC()
        return "{}{}{}".format(self.OKBLUE,msg,self.ENDC)
    
    def okcyan(msg):
        self = ESC()
        return "{}{}{}".format(self.OKCYAN,msg,self.ENDC)
    
    def okgreen(msg):
        self = ESC()
        return "{}{}{}".format(self.OKGREEN,msg,self.ENDC)
    
    def warning(msg):
        self = ESC()
        return "{}{}{}".format(self.WARNING,msg,self.ENDC)
    
    def fail(msg):
        self = ESC()
        return "{}{}{}".format(self.FAIL,msg,self.ENDC)

    def bold(msg):
        self = ESC()
        return "{}{}{}".format(self.BOLD,msg,self.ENDC)
    
    def underline(msg):
        self = ESC()
        return "{}{}{}".format(self.UNDERLINE,msg,self.ENDC)