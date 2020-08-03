import numpy as np
from PIL import Image

data = open("utah_pit_color_data.txt","r")
line = data.readline()
colors = np.array([])
cnt = 1
commas =0
while line:
    for c in line:
        if c == ',':
            commas +=1
    #line = line.replace(',','')
    #line = line.replace('\n','')
    #line = line.strip()
    #color = np.fromstring(line,sep=' ')
    #colors = np.append(colors,color,axis=0)

    line = data.readline()
    cnt += 1
print(commas)
#colors = np.append(colors,np.zeros((856,)),axis=0)
#print(np.shape(colors))
#colors = np.reshape(colors,(int(np.sqrt(np.shape(colors)[0]/3)),int(np.sqrt(np.shape(colors)[0]/3)),3))
#print(colors.shape)
#img = Image.fromarray(colors, 'RGB')
#img.save('utah_texture.png')
#img.show()
data.close()