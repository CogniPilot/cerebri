from IPython.display import Image, display
import os
import tempfile
from casadi import print_operator as print_operator_orig
import casadi.tools.graph as mod_graph

def dotdraw(pdot, width=1920, direction='LR'):
  curdir = os.getcwd()
  # dot draw creates a source.dot file, lets move to the tmp directory
  os.chdir(tempfile.gettempdir())
  graph = mod_graph.dotgraph(pdot, direction=direction)
  png = graph.create_png()
  os.chdir(curdir)
  return Image(png, width=width)

def print_operator_escaped(expr, strs):
  # replace <f> with {f} to get rid of < or >
  strs = [ si.replace('<', '{').replace('>', '}') for si in strs ]
  s = print_operator_orig(expr, strs)
  # escape < or >
  s = s.replace('<', '\<').replace('>', '\>')
  # replace {f} with <f>
  s = s.replace('{', '<').replace('}', '>')
  return s

# monkey patch
mod_graph.print_operator = print_operator_escaped