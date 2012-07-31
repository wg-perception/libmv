#!/usr/bin/python

# Example for:
#   void NormalizedEightPointSolver(const Mat &x1, const Mat &x2, Mat3 *F);

func = 'NormalizedEightPointSolver'

input_type = ['Mat', 'Mat']
input      = ['x1', 'x2']

output_type = ['Mat3']
output      = ['F']


# ---------------------------

# function name
# OpenCV name convention: first caracter is small capital
func2 = func[0].lower() + func[1:]



# ---------------------------

# args
arguments = "( "
n = len(input)
spaces = ' ' * len(func)
for i in xrange(n):
    arguments += "const Mat &" + input[i]
    if i < n-1:
        arguments += ",\n  " + spaces

n = len(output)
if n > 0:
    arguments += ",\n  " + spaces
    for i in xrange(n):
        arguments += "Mat &" + output[i]
        if i < n-1:
            arguments += ",\n  " + spaces
            
arguments += " )"
#print arguments
arguments01 = arguments


arguments = "( "
n = len(input)
for i in xrange(n):
    arguments += "const Mat &_" + input[i]
    if i < n-1:
        arguments += ",\n  " + spaces

n = len(output)
if n > 0:
    arguments += ",\n  " + spaces
    for i in xrange(n):
        arguments += "Mat &_" + output[i]
        if i < n-1:
            arguments += ",\n  " + spaces
        
arguments += " )"
#print arguments
arguments02 = arguments


# ---------------------------

# types difinition

# dictionary with multiple values
l = zip(input_type+output_type, input+output)
d={}
for key, val in l:
    d.setdefault(key, []).append(val)
#print d

types = ""
for key in d.keys():
    line = "    libmv::" + key + " "
    for val in d[key]:
        line += val + ", "
    line = line[0:-2]
    types += line + ";\n"
#print types


# ---------------------------

# cv2eigen
cv2eigen = ""
n = len(input)
for i in xrange(n):
    cv2eigen += "    cv2eigen( _" + input[i] + ", " + input[i]+ " );\n"
#print cv2eigen

# eigen2cv
eigen2cv = ""
n = len(output)
for i in xrange(n):
    eigen2cv += "    eigen2cv( " + output[i] + ", _" + output[i]+ " );\n"
#print eigen2cv


# ---------------------------

# call function
call_func ="    libmv::" + func + "( "
n = len(input)
for i in xrange(n):
    call_func += input[i]
    if i < n-1:
        call_func += ", "

    
n = len(output)
if n > 0:
    call_func += ", "
for i in xrange(n):
    call_func += "&" + output[i]
    if i < n-1:
        call_func += ", "
call_func += " );\n"


# ---------------------------

# Check types
depth = "    int depth = " + input[0] +".depth();"

n = len(input)
if n > 1:
    depth += "\n    CV_Assert("
    for i in xrange(1,n):
        depth += " depth == " + input[i] + ".depth()"
        if i < n-1:
            depth += " &&"
    depth += " );\n"


# ---------------------------

# list of arguments (caller function)
arguments = "( "
n = len(input)
for i in xrange(n):
    arguments += input[i]
    if i < n-1:
        arguments += ", "

n = len(output)
if n > 0:
    arguments += ", "    
    for i in xrange(n):
        arguments += output[i]
        if i < n-1:
            arguments += ", "

arguments += " )"



# ---------------------------

# colors
CSI      = "\x1B["
RED      = CSI+"31;40m"
GREEN    = CSI+"32;40m"
YELLOW   = CSI+"33;40m"
BLUE     = CSI+"34;40m"
MAGENTA  = CSI+"35;40m"
CYAN     = CSI+"36;40m"
RESET    = CSI+"0m"



# ===========================


print "\n" + GREEN + "Wrapper (in .cpp)\n===============\n" + RESET

wrapper_func  = "template<typename T>\nvoid\n" + GREEN + func2 + RESET + arguments02
wrapper_func += "\n{\n" + types + "\n" + cv2eigen + "\n"
wrapper_func += YELLOW + call_func + RESET + "\n" + eigen2cv + "}\n"
print wrapper_func

api_func = "void\n" + GREEN + func2 + RESET + arguments01 + "\n{\n" + depth
api_func += "\n    if( depth == CV_32F )\n    {\n"
api_func += "    " + "    // " + func2 + "<" + RED + "float" + RESET + ">" + arguments + ";\n"
api_func += "        std::cerr << \"Function " + func2 + " not handled for float\" << std::endl;"
api_func += "\n    }\n"
api_func += "    else\n    {\n"
api_func += "    " + "    " + func2 + "<" + RED + "double" + RESET + ">" + arguments + ";"
api_func += "\n    }\n"
api_func += "}\n"
print api_func




print "\n\n" + CYAN + "Prototype (in .hpp)\n===================\n" + RESET

prototype = RED + "CV_EXPORTS\n" + RESET
prototype += "void\n" +  CYAN + func2 + RESET + arguments01 + ";\n"
print prototype
