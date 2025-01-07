from assets.load import load_assembly
from assets.save import clear_saved_sdfs
from examples.run_joint_plan import get_planner
from itertools import chain, combinations
from argparse import ArgumentParser
from copy import deepcopy
import os
import networkx as nx
import sys
import threading
import queue

parser = ArgumentParser()
parser.add_argument('--id', type=str, required=True, help='assembly id (e.g. 00000)')
parser.add_argument('--dir', type=str, default='multi_assembly', help='directory storing all assemblies')
parser.add_argument('--rotation', default=False, action='store_true')
parser.add_argument('--planner', type=str, required=True, choices=['bfs', 'bk-rrt'])
parser.add_argument('--body-type', type=str, default='sdf', choices=['bvh', 'sdf'], help='simulation type of body')
parser.add_argument('--sdf-dx', type=float, default=0.05, help='grid resolution of SDF')
parser.add_argument('--collision-th', type=float, default=1e-2)
parser.add_argument('--force-mag', type=float, default=100, help='magnitude of force')
parser.add_argument('--frame-skip', type=int, default=100, help='control frequency')
parser.add_argument('--seq-max-time', type=float, default=3600, help='sequence planning timeout')
parser.add_argument('--path-max-time', type=float, default=120, help='path planning timeout')
parser.add_argument('--seed', type=int, default=1, help='random seed')
parser.add_argument('--render', default=False, action='store_true', help='if render the result')
parser.add_argument('--record-dir', type=str, default=None, help='directory to store rendering results')
parser.add_argument('--save-dir', type=str, default=None)
parser.add_argument('--n-save-state', type=int, default=100)
args = parser.parse_args()

project_base_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '.'))
asset_folder = os.path.join(project_base_dir, './assets')
assembly_dir = os.path.join(asset_folder, args.dir, args.id)

#if args.rotation: args.seq_max_time *= 2
        
if args.record_dir is None:
    record_path = None

meshes, names = load_assembly(assembly_dir, return_names=True)
print(names)

#import graph_tool.all as gt

#create Graph



#objects = chain.from_iterable(combinations(part_ids, r) for r in range(1, len(part_ids)+1))
#print(len(list(objects)))
#graph.add_nodes_from(objects)
#print("f1")

def addChildNodes(parentNode):
    print(parentNode)
    if(len(parentNode)==0):return
    newNodes = list(combinations(parentNode, len(parentNode)-1))
    print(newNodes)
    graph.add_nodes_from(newNodes)
    for newNode in newNodes:
        graph.add_edge(parentNode,newNode)
        addChildNodes(newNode)


graph = nx.DiGraph()
part_ids = [name.replace('.obj', '') for name in names]
graph.add_node(tuple(part_ids))

#objects = chain.from_iterable(combinations(part_ids, r) for r in range(1, len(part_ids)+1))

#addChildNodes(tuple(part_ids))

#print(len(list(objects)))


import sys
#sys.exit()





#alternative:

import concurrent.futures

maxWorkers=16
executer = concurrent.futures.ThreadPoolExecutor(max_workers=maxWorkers)
finishList = []

nodeQueue = [part_ids]
maxSize = 2**(len(part_ids))
n=0
finished=False

def checkObject(objects, object):
    newNode = objects.copy()
    newNode.remove(object)
    #TODO: check if object can be removed from Objects
    newNodeTuple = tuple(newNode)
    if not (newNodeTuple in graph):
        graph.add_node(newNodeTuple)
        if (len(newNode) > 0):
            nodeQueue.append(newNode)
    graph.add_edge(tuple(objects), newNodeTuple, moveID=object, stillIDs=newNode)

futures=[]

while True:
    #print(len(nodeQueue))
    if (len(nodeQueue)>0):
        objects = nodeQueue.pop()
        #print(objects)
        for object in objects:
            futures.append(executer.submit(checkObject, objects, object))
    else:
        for future in futures:
            if future.done():
                futures.remove(future)
        if len(futures)==0:
            break


executer.shutdown(wait = True)
#print("hi")
for node in graph.nodes:
    print(node)
#wait until all executers are finished





#for edge in graph.edges:
#    print(graph[edge[0]][edge[1]])
    
#import matplotlib.pyplot as plt
#nx.draw(graph, with_labels=True)
#plt.show()