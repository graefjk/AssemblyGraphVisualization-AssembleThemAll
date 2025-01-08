from assets.load import load_assembly
from assets.save import clear_saved_sdfs
from examples.run_joint_plan import get_planner
from itertools import chain, combinations
from argparse import ArgumentParser
from copy import deepcopy
import os
import networkx as nx
import concurrent.futures
import sys
import threading
import queue
import time
from shutil import make_archive

start = time.time()

parser = ArgumentParser()
parser.add_argument('--planner', type=str, required=True, choices=['bfs', 'bk-rrt'])
parser.add_argument('--id', type=str, required=True, help='assembly id (e.g. 00000)')
parser.add_argument('--dir', type=str, default='multi_assembly', help='directory storing all assemblies')
parser.add_argument('--move-id', type=str, default='0')
parser.add_argument('--still-ids', type=str, nargs='+', default=['1'])
parser.add_argument('--rotation', default=False, action='store_true')
parser.add_argument('--body-type', type=str, default='sdf', choices=['bvh', 'sdf'], help='simulation type of body')
parser.add_argument('--sdf-dx', type=float, default=0.05, help='grid resolution of SDF')
parser.add_argument('--collision-th', type=float, default=1e-2)
parser.add_argument('--force-mag', type=float, default=100, help='magnitude of force')
parser.add_argument('--frame-skip', type=int, default=100, help='control frequency')
parser.add_argument('--max-time', type=float, default=120, help='timeout')
parser.add_argument('--seed', type=int, default=1, help='random seed')
parser.add_argument('--render', default=False, action='store_true', help='if render the result')
parser.add_argument('--record-dir', type=str, default=None, help='directory to store rendering results')
parser.add_argument('--save-dir', type=str, default=None)
parser.add_argument('--n-save-state', type=int, default=100)
parser.add_argument('--save-sdf', default=False, action='store_true')


args = parser.parse_args()

project_base_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '.'))
asset_folder = os.path.join(project_base_dir, './assets')
assembly_dir = os.path.join(asset_folder, args.dir, args.id)
filename = 'assembly_' + str(args.id)
save_folder = os.path.join(project_base_dir, './assemblies', './' + filename)
os.makedirs(save_folder)

#if args.rotation: args.seq_max_time *= 2
        
if args.record_dir is None:
    record_path = None

meshes, names = load_assembly(assembly_dir, return_names=True)

clear_saved_sdfs(assembly_dir)


print(names)


graph = nx.DiGraph()
part_ids = [name.replace('.obj', '') for name in names]
graph.add_node(tuple(part_ids))

maxWorkers = None #set to None to set to the number of processors on the machine 
executer = concurrent.futures.ProcessPoolExecutor(max_workers=maxWorkers)
finishList = []

nodeQueue = [part_ids]
maxSize = 2**(len(part_ids))
n=0
finished=False

def checkObject(objects, object, graph, nodeQueue):
    newNode = objects.copy()
    newNode.remove(object)
    print("checking ",newNode, object)
    newNodeTuple = tuple(newNode)
    #check if object can be removed from Objects
    if(len(newNode)>0):
        planner = get_planner(args.planner)(asset_folder, assembly_dir, object, newNode, args.rotation, args.body_type, args.sdf_dx, args.collision_th, args.force_mag, args.frame_skip, args.save_sdf)
        status, t_plan, path = planner.plan(args.max_time, seed=args.seed, return_path=True, render=args.render, record_path=record_path)
        print("result:",newNode, object, status)
        if status != 'Success':
            return
        step_folder = os.path.join(save_folder, './steps/' ,  "./"+str(newNode)+"_" +str(object))
        os.makedirs(step_folder)
        save_dir = os.path.join(step_folder, './matrixes')
        planner.save_path(path, save_dir, args.n_save_state)

    return newNode, newNodeTuple, tuple(objects), object 
    
    

#tests
#planner = get_planner(args.planner)(asset_folder, assembly_dir, part_ids[0], part_ids[1:], args.rotation, args.body_type, args.sdf_dx, args.collision_th, args.force_mag, args.frame_skip, args.save_sdf)
#status, t_plan, path = planner.plan(args.max_time, seed=args.seed, return_path=True, render=args.render, record_path=record_path)

futures=[]

while True:
    #print(len(nodeQueue))
    if (len(nodeQueue)>0):
        objects = nodeQueue.pop()
        #print(objects)
        for object in objects:
            futures.append(executer.submit(checkObject, objects, object, graph,nodeQueue))
    else:
        for future in futures:
            if future.done():
                result = future.result()
                if result is not None:
                    newNode, newNodeTuple, objects, object = result
                    if not (newNodeTuple in graph):
                        graph.add_node(newNodeTuple)
                        if (len(newNode) > 0):
                            nodeQueue.append(newNode)
                            #print("appending ",newNode, nodeQueue)
                    graph.add_edge(tuple(objects), newNodeTuple, moveID=object, stillIDs=newNode)

                futures.remove(future)
        if (len(futures)==0) and (len(nodeQueue)==0):
            print("breaking ", nodeQueue)
            break

executer.shutdown(wait = True)

import json
import shutil

with open(os.path.join(save_folder,'./graph.json'), 'w') as f:
    json.dump(nx.readwrite.json_graph.node_link_data(graph), f)

shutil.make_archive(filename, 'zip', save_folder)
shutil.rmtree(save_folder)

for node in graph.nodes:
    print(node)



print("time:", time.time()-start,"s")

#for edge in graph.edges:
#    print(graph[edge[0]][edge[1]])
    
#import matplotlib.pyplot as plt
#nx.draw(graph, with_labels=True)
#plt.show()