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
'''
cache: list
origin_pro: farthest projection to the perspective
x,y,z: coordinates of cube center
dist: distance to the center of voxel
cube_pixel: propotion of pixel to one cube side
cube_width:
'''
def get_projection(cache,origin_pro,x,dist,cube_pixel,cube_width):
    alpha=0.05 # parameter remain to be judged
    s=int(math.floor((x-alpha)*100))
    e=int(math.floor((x+alpha)*100))
    m=int(math.floor(x*100))
    # for i in range(m,e):
    #     if i in cache:
    #         return cache[i]
    # for i in range(s,m):
    #     if i in cache:
    #         return cache[i]
    proportion=(x+dist)/(dist+math.sqrt(2)*cube_width/2)
    new_width=int(origin_pro.size[0]*proportion)
    new_projection=origin_pro.resize((new_width, new_width))
    padding = int((origin_pro.size[0] - new_width) / 2)
    new_projection_data=np.asarray(new_projection)
    if new_width&1==1:
        new_projection_data=np.pad(new_projection_data,((padding,padding+1),(padding,padding+1)),'constant')
    else:
        new_projection_data=np.pad(new_projection_data,((padding,padding),(padding,padding)),'constant')
    return new_projection_data

def modify_mask(cube_pixel,mask,target_width):
    threshold=int(cube_pixel*cube_pixel/2)
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
'''
Algorithm steps:
for theta in R:
    1.project on the far sceen
    2.calculate projection of voxel for the cube, if distance of two voxel center
      is less than 1/2, then get projection from the cache
      else, caculate new projection using resize() and store it into the cache
    3.with the projection, generate the mask, set all the voxel out side of the
      mask into False
'''

if __name__=='__main__':
    # init parameters
    pic_length = 150
    w = 100
    l = 100
    h = 100
    dist_to_center = w * 3 / 2
    cubes = []
    cube_pixel = int(pic_length * 2 / w)
    cache = {}
    radians=0
    should_trans = False

    # image_list=get_files_in_dir('./Tank')
    image_list=get_files_in_dir('./Bunny')
    # image_list=get_files_in_dir('./Teddy')

    for i in range(0,30):
        cube = generate_cube(w, l, h)
        img = Image.open(image_list[i])
        img = img.convert('L')
        img_size = img.size
        cropped_image = img.crop([img_size[0] / 2 - pic_length, img_size[1] / 2 - pic_length, img_size[0] / 2 + pic_length,
                                  img_size[1] / 2 + pic_length])
        cropped_image = ImageOps.invert(cropped_image)
        cropped_image = cropped_image.rotate(-90)
        theta = math.radians(radians)
        radians = radians + 3
        R_Matrix = np.array([[math.cos(theta), math.sin(theta)],
                             [-math.sin(theta), math.cos(theta)]])
        print(i)
        print(should_trans)
        print(radians)
        pro = cropped_image
        data = np.asarray(pro)
        data = modify_mask(cube_pixel, data, pic_length * 2)
        data = np.int8(data > 0)
        for x in range(0,w):
            for y in range(0,l):
                transformed_x = x - w / 2
                transformed_y = y - l / 2
                pos = np.array([transformed_x, transformed_y])
                pos = pos.dot(R_Matrix)
                for z in range(0,h):
                    temp_y=int(math.floor(pos[1]+l/2))
                    if temp_y<0:
                        cube[x][y][z]=False
                    if temp_y>=w:
                        temp_y=w-1
                    if cube[x][y][z]==False:
                        cube[x][y][z]=data[temp_y][z]
        cubes.append(cube)
    radians=0
    for i in range(119,89,-1):
        cube = generate_cube(w, l, h)
        img = Image.open(image_list[i])
        img = img.convert('L')
        img_size = img.size
        cropped_image = img.crop(
            [img_size[0] / 2 - pic_length, img_size[1] / 2 - pic_length, img_size[0] / 2 + pic_length,
             img_size[1] / 2 + pic_length])
        cropped_image = ImageOps.invert(cropped_image)
        cropped_image = cropped_image.rotate(-90)
        theta = math.radians(radians)
        radians = radians - 3
        R_Matrix = np.array([[math.cos(theta), math.sin(theta)],
                             [-math.sin(theta), math.cos(theta)]])
        print(i)
        print(should_trans)
        print(radians)
        pro = cropped_image
        data = np.asarray(pro)
        data = modify_mask(cube_pixel, data, pic_length * 2)
        data = np.int8(data > 0)
        for x in range(0, w):
            for y in range(0, l):
                transformed_x = x - w / 2
                transformed_y = y - l / 2
                pos = np.array([transformed_x, transformed_y])
                pos = pos.dot(R_Matrix)
                for z in range(0, h):
                    temp_y = int(math.floor(pos[1] + l / 2))
                    if temp_y < 0:
                        cube[x][y][z] = False
                    if temp_y >= w:
                        temp_y = w - 1
                    if cube[x][y][z] == False:
                        cube[x][y][z] = data[temp_y][z]
        cubes.append(cube)
    # logical and ops between squares
    cube1=cubes[0]
    for i in range(0,30):
        cube1=np.logical_and(cube1,cubes[i])
    cube2 = cubes[30]
    for i in range(30,60):
        cube2=np.logical_and(cube2,cubes[i])
    rst1=generate_cube(w,l,h)
    for y in range(0,int(l/2)):
        for x in range(0,w):
            rst1[x][y]=cube1[x][y]
    for y in range(int(l/2),l):
        for x in range(0,w):
            rst1[x][y]=cube2[x][y]
    # show_cube(rst1)

    # Generate back img
    cubes.clear()
    radians=0
    for i in range(60, 90):
        cube = generate_cube(w, l, h)
        img = Image.open(image_list[i])
        img = img.convert('L')
        img_size = img.size
        cropped_image = img.crop(
            [img_size[0] / 2 - pic_length, img_size[1] / 2 - pic_length, img_size[0] / 2 + pic_length,
             img_size[1] / 2 + pic_length])
        cropped_image = ImageOps.invert(cropped_image)
        cropped_image = cropped_image.rotate(-90)
        theta = math.radians(radians)
        radians = radians + 3
        R_Matrix = np.array([[math.cos(theta), math.sin(theta)],
                             [-math.sin(theta), math.cos(theta)]])
        print(i)
        print("radians", radians)
        pro = cropped_image
        data = np.asarray(pro)
        data = modify_mask(cube_pixel, data, pic_length * 2)
        data = np.int8(data > 0)
        for x in range(0, w):
            for y in range(0, l):
                transformed_x = x - w / 2
                transformed_y = y - l / 2
                pos = np.array([transformed_x, transformed_y])
                pos = pos.dot(R_Matrix)
                for z in range(0, h):
                    temp_y = int(math.floor(pos[1] + l / 2))
                    if temp_y < 0:
                        cube[x][y][z] = False
                    if temp_y >= w:
                        temp_y = w - 1
                    if cube[x][y][z] == False:
                        cube[x][y][z] = data[temp_y][z]
        cubes.append(cube)
    radians = 0
    for i in range(59, 29, -1):
        cube = generate_cube(w, l, h)
        img = Image.open(image_list[i])
        img = img.convert('L')
        img_size = img.size
        cropped_image = img.crop(
            [img_size[0] / 2 - pic_length, img_size[1] / 2 - pic_length, img_size[0] / 2 + pic_length,
             img_size[1] / 2 + pic_length])
        cropped_image = ImageOps.invert(cropped_image)
        cropped_image = cropped_image.rotate(-90)
        theta = math.radians(radians)
        radians = radians - 3
        R_Matrix = np.array([[math.cos(theta), math.sin(theta)],
                             [-math.sin(theta), math.cos(theta)]])
        print(i)
        print("radians", radians)
        pro = cropped_image
        data = np.asarray(pro)
        data = modify_mask(cube_pixel, data, pic_length * 2)
        data = np.int8(data > 0)
        for x in range(0, w):
            for y in range(0, l):
                transformed_x = x - w / 2
                transformed_y = y - l / 2
                pos = np.array([transformed_x, transformed_y])
                pos = pos.dot(R_Matrix)
                for z in range(0, h):
                    temp_y = int(math.floor(pos[1] + l / 2))
                    if temp_y < 0:
                        cube[x][y][z] = False
                    if temp_y >= w:
                        temp_y = w - 1
                    if cube[x][y][z] == False:
                        cube[x][y][z] = data[temp_y][z]
        cubes.append(cube)
    # logical and ops between squares
    cube3=cubes[0]
    for i in range(0,30):
        cube3=np.logical_and(cube3,cubes[i])
    cube4 = cubes[30]
    for i in range(30,60):
        cube4=np.logical_and(cube4,cubes[i])
    rst2=generate_cube(w,l,h)
    for y in range(0,int(l/2)):
        for x in range(0,w):
            rst2[x][y]=cube3[x][y]
    for y in range(int(l/2),l):
        for x in range(0,w):
            rst2[x][y]=cube4[x][y]
    # show_cube(rst2)
    # integrate full image
    for x in range(int(w/2),w):
        for y in range(0,l):
            rst1[x][y]=rst2[w-1-x][w-1-y]

    rst1=get_crust(rst1)
    show_cube(rst1)
    np.save("cube_Bunny_100.npy", rst1)