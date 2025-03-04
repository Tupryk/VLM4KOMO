import robotic as ry
import numpy as np
import time
from robotic import SimulationEngine
import re
import ast

line = 'place(4,   1.23)'

objectives=[]

if line.startswith("place"):
    match = re.match(r'\s*place\s*\(\s*(.+?)\s*,\s*(.+?)\s*(?:,\s*(.+?))?\s*\)\s*', line)

    if match:
        x = ast.literal_eval(match.group(1).strip())
        y = ast.literal_eval(match.group(2).strip())

        # Handle the optional z parameter
        z = ast.literal_eval(match.group(3).strip()) if match.group(3) else 0.0

        objective_dict = {"feature": "place", "target": [x,y], "z": z}
        objectives.append(objective_dict)

    else:
        raise ValueError("String format does not match.")




print(objectives)