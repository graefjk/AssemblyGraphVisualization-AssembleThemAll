from assets.load import load_assembly
from assets.save import clear_saved_sdfs
from examples.run_joint_plan import get_planner
from argparse import ArgumentParser
import os
import networkx as nx
import concurrent.futures
import time
import json
import shutil
import psutil
from collections import deque 
import logging
logging.disable(logging.WARNING)

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
parser.add_argument('--check-subsets', type=int, default=1)
parser.add_argument('--check-failcontacts', type=int, default=1)
parser.add_argument('--top-down', type=int, default=1)



args = parser.parse_args()
print(not args.top_down)
print(bool(args.check_subsets))

project_base_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '.'))
asset_folder = os.path.join(project_base_dir, './assets')
assembly_dir = os.path.join(asset_folder, args.dir, args.id)
filename = 'assembly_' + str(args.id)
save_folder = os.path.join(project_base_dir, './assemblies', './' + filename)
os.makedirs(save_folder)
objects_folder = os.path.join(save_folder, './objects')
os.makedirs(objects_folder)
for file in os.listdir(assembly_dir):
    shutil.copy2(os.path.join(assembly_dir,file), objects_folder)

#if args.rotation: args.seq_max_time *= 2
        
if args.record_dir is None:
    record_path = None

meshes, names = load_assembly(assembly_dir, return_names=True)

clear_saved_sdfs(assembly_dir)

print(names)


graph = nx.DiGraph()
part_ids = sorted([name.replace('.obj', '') for name in names])

if args.top_down:
    graph.add_node(tuple(sorted(part_ids)))

maxWorkers = os.cpu_count()
executer = concurrent.futures.ProcessPoolExecutor(max_workers=maxWorkers)
finishList = []

setList = [[] for i in range(len(part_ids))]
contactIDsList = [[frozenset(part_ids)] for i in range(len(part_ids))]
if args.top_down:
    nodeQueue = deque([part_ids])
else:
    nodeQueue = deque([None])

maxSize = 2**(len(part_ids))
finished=False

futures=[]
successes = 0
subsetSuccesses = 0
timeouts = 0
failures = 0
n = 0

def checkObject(objects, object, id):
    newNode = sorted(objects.copy())
    newNode.remove(object)
    print("checking ",newNode, object)
    newNodeTuple = tuple(newNode)
    status = 'Success'
    #check if object can be removed from Objects
    if(len(newNode)>0):
        planner = get_planner(args.planner)(asset_folder, assembly_dir, object, newNode, args.rotation, args.body_type, args.sdf_dx, args.collision_th, args.force_mag, args.frame_skip, args.save_sdf)
        status, t_plan, path, contactIDs = planner.plan(args.max_time, seed=args.seed, return_path=True, render=args.render, record_path=record_path)
        print("result:",newNode, object, status)
        if status != 'Success':
            return None, None, None, object, status, id, contactIDs
        step_folder = os.path.join(save_folder, './steps/' ,  "./"+str(id))
        planner.save_path(path, step_folder, args.n_save_state)

    return newNode, newNodeTuple, objects, object, status, id, set()
    

def getSetID(objects, object):
    global subsetSuccesses
    if len(setList[int(object)]) > 0:
        for item in setList[int(object)]:
            if(set(objects).issubset(item[0])):
                if len(objects) < len(item[0]):
                    print("result:" , list(set(objects)-set(object)), object, "is a subset of", item[0] ,"Success")
                    subsetSuccesses +=1
                    return item[1]
                return -1
    return None
    

#tests
#planner = get_planner(args.planner)(asset_folder, assembly_dir, part_ids[0], part_ids[1:], args.rotation, args.body_type, args.sdf_dx, args.collision_th, args.force_mag, args.frame_skip, args.save_sdf)
#status, t_plan, path = planner.plan(args.max_time, seed=args.seed, return_path=True, render=args.render, record_path=record_path)


def appendNodeToQueue(newNode, objects):
    if args.top_down and (len(newNode) > 0):
        nodeQueue.appendleft(newNode)
    elif not args.top_down:
        nodeQueue.appendleft(objects)

def addNodeToGraphAndQueue(newNode, objects):
    if args.top_down and not (tuple(sorted(newNode)) in graph):
        graph.add_node(tuple(sorted(newNode)))
        appendNodeToQueue(newNode, objects)
    elif not args.top_down and not (tuple(sorted(objects)) in graph):
        graph.add_node(tuple(sorted(objects)))
        appendNodeToQueue(newNode, objects)
        

'''''
tries to create an Edge by removing object from objects
'''''
def tryCreateEdge(object, objects):
    global failures, n, successes, executer, futures, contactIDsList

    newNode = objects.copy()
    newNode.remove(object)  
    if args.check_subsets:
        matrixID = getSetID(objects, object)
        if matrixID == -1:
            return
        elif matrixID is not None:
            addNodeToGraphAndQueue(newNode, objects)
            graph.add_edge(tuple(sorted(newNode)), tuple(sorted(objects)), moveID=object, edgeID=matrixID)
            return 
        
    isSuperset = False
    if args.check_failcontacts:
        for idList in contactIDsList[int(object)]:
            if set(newNode).issuperset(idList):
                isSuperset = True
    if not isSuperset:
        futures.append(executer.submit(checkObject, objects, object, n))
    else:
        print("result:",newNode, object, "Failure")
        failures+=1
    n += 1


def checkFutures():
    global  successes, timeouts, futures, contactIDsList

    for future in futures:
        if future.done():
            newNode, newNodeTuple, objects, object, status, id, contactIDs = future.result()
            contactIDs = list(contactIDs)
            for i in range(len(contactIDs)):
                contactIDs[i] = contactIDs[i][4:] #remove 'part' from e.g. 'part3'
            contactIDs = frozenset(contactIDs)
            if newNode is not None:
                addNodeToGraphAndQueue(newNode, objects)
                graph.add_edge(newNodeTuple, tuple(sorted(objects)), moveID=object, edgeID=id)
                if getSetID(objects, object) is None:
                    setList[int(object)].append([set(objects), id])
            futures.remove(future)
            if status == 'Success':
                successes+=1
            elif args.check_failcontacts:
                isSuperset = False
                for i in range(len(contactIDsList[int(object)])):
                    if contactIDs.issubset(contactIDsList[int(object)][i]):
                        contactIDsList[int(object)][i]=contactIDs
                    elif contactIDs.issuperset(contactIDsList[int(object)][i]):
                        isSuperset = True         
                if not isSuperset and not (contactIDs in contactIDsList[int(object)]):
                    contactIDsList[int(object)].append(contactIDs)
                    print(contactIDsList)
                contactIDsList[int(object)] = list(set(contactIDsList[int(object)])) #remove duplicates
            else:
                timeouts+=1


#Problem: bottom up algorithm checks some edges multiple times

while True:
    #print(len(futures))
    #print("RAM: ",psutil.virtual_memory().percent)
    
    if (psutil.virtual_memory().percent > 80): #clear RAM
        print("clearing objects from RAM")
        executer.shutdown(wait = True)
        executer = concurrent.futures.ProcessPoolExecutor(max_workers=maxWorkers)

    while (len(nodeQueue)>0) and (len(futures)< maxWorkers*2):
        #print(len(futures), maxWorkers*2)
        objects = nodeQueue.popleft()
        print("submitting", objects)
        if args.top_down:
            for object in objects:
                tryCreateEdge(object, objects)
        else:
            if objects is None:
                objects = []
            nodesToAdd = set(part_ids) - set(objects)
            for node in nodesToAdd:
                allParts = objects.copy()
                allParts.append(node)
                tryCreateEdge(node, allParts)

    
    checkFutures()

    if (len(futures)==0) and (len(nodeQueue)==0):
        print("breaking ", nodeQueue)
        break

    time.sleep(1) # only check once every second to not block the thread
    


executer.shutdown(wait = True)

print("writing graph.json")
with open(os.path.join(save_folder,'./graph.json'), 'w') as f:
    json.dump(nx.readwrite.json_graph.node_link_data(graph), f)

print("packing zip file")
shutil.make_archive(filename, 'zip', save_folder)
shutil.rmtree(save_folder)

for node in graph.nodes:
    print(node)

print("successes:", successes, "subSets found:" , subsetSuccesses ,"timeouts:", timeouts, "failures:",failures ,"time:", time.time()-start, "s")

#for edge in graph.edges:
#    print(graph[edge[0]][edge[1]])
    
#import matplotlib.pyplot as plt
#nx.draw(graph, with_labels=True)
#plt.show()