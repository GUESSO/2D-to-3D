# --encoding: utf-8--
import os

import numpy as np
import math
from PIL import Image,ImageOps
import matplotlib.pyplot as plt
# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

def generate_cube(w,l,h):
    x, y, z = np.indices((w, l, h))
    # cube = (x < w) & (y < l) & (z < h)
    cube = (x > w) & (y > l) & (z > h)
    return cube

def get_projection(cache,origin_pro,x,dist,cube_pixel,cube_width):
    alpha=0.05 # parameter remain to be judged
    s=int(math.floor((x-alpha)*100))
    e=int(math.floor((x+alpha)*100))
    m=int(math.floor(x*100))
    proportion=(x+dist)/(dist+math.sqrt(2)*cube_width/2)
    new_width=int(origin_pro.size[0]*proportion)
    new_projection=origin_pro.resize((new_width, new_width))
    padding = int((origin_pro.size[0] - new_width) / 2)
    new_projection_data=np.asarray(new_projection)
    if new_width&1==1:
        new_projection_data=np.pad(new_projection_data,((padding,padding+1),(padding,padding+1)),'constant')
    else:
        new_projection_data=np.pad(new_projection_data,((padding,padding),(padding,padding)),'constant')
    # new_projection=Image.fromarray(new_projection_data)

    # cache[x*100]=new_projection_data
    return new_projection_data

def modify_mask(cube_pixel,mask,target_width):
    # threshold=1
    threshold=cube_pixel*cube_pixel/2
    rst=[]
    for i in range(0,target_width,cube_pixel):
        temp=[]
        for j in range(0,target_width,cube_pixel):
            kernel=mask[i:i+cube_pixel]
            kernel=kernel[:,j:j+cube_pixel]
            if np.sum(kernel)>threshold:
                temp.append(255)
            else:
                temp.append(0)
        rst.append(temp)
    rst=np.array(rst)
    return rst

def get_files_in_dir(file_dir):
    L=[]
    for root,dirs,files in os.walk((file_dir)):
        for file in files:
            L.append(os.path.join(root,file))
    return L

def show_cube(cube):
    colors = np.empty(cube.shape, dtype=object)
    colors[cube] = 'yellow'

    # and plot everything
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    # ax.voxels(cube, facecolors=colors, edgecolor='k')
    ax.voxels(cube, facecolors=colors, edgecolor='None')
    ax.view_init(elev=0, azim=0)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.grid('off')
    # plt.axis('off')
    plt.show()

def get_crust(cube):
    dir_x=[1,0,0,0,0,-1]
    dir_y=[0,1,-1,0,0,0]
    dir_z=[0,0,0,1,-1,0]
    w=len(cube)
    cube_rst=generate_cube(w,w,w)

    for x in range(0,w):
        for y in range(0,w):
            for z in range(0,w):
                if cube[x][y][z] == False:
                    continue
                for ind in range(0,6):
                    temp_x=x+dir_x[ind]
                    if temp_x<0 or temp_x>=w:
                        continue
                    temp_y=y+dir_y[ind]
                    if temp_y<0 or temp_y>=w:
                        continue
                    temp_z=z+dir_z[ind]
                    if temp_z<0 or temp_z>=w:
                        continue
                    if cube[temp_x][temp_y][temp_z]==False:
                        cube_rst[x][y][z]=True
                        break
    return cube_rst

if __name__=="__main__":
    cube=np.load('cube_Bunny_100.npy')
    print(np.sum(cube))
    show_cube(cube)
    cube=get_crust(cube)
    print(np.sum(cube))
    show_cube(cube)