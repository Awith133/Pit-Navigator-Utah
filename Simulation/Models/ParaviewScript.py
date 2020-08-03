from paraview.simple import *

parameter = 'round3_batch3.ply'

#file=OpenDataFile('/home/alex/pit-navigator-utah/Simulation/Models/' +parameter)
#Show(file)
ResetCamera()
Render()
camera=GetActiveCamera()

#pan to middle
pos = camera.GetPosition()
focal = camera.GetFocalPoint()
pos = (pos[0],pos[1]-4,pos[2])
focal = (pos[0],pos[1],focal[2])
camera.SetPosition(pos)
camera.SetFocalPoint(focal)
Render()


camera.Elevation(-90)
rep = GetDisplayProperties()
rep.DiffuseColor = [0.5,0.5,0.5]
Render()
for i in range(120):
    camera.Yaw(3)
    ResetCamera()
    SaveScreenshot('/home/alex/pit-navigator-utah/Simulation/Models/Movies/'+parameter[:-4]+'_Images/'+parameter[:-4]+'_1_{0}.png'.format(str(i).zfill(3)))
for i in range(30):
    camera.Pitch(-3)
    ResetCamera()
    if i > 28:
        camera.Roll(90)
        ResetCamera()
    SaveScreenshot('/home/alex/pit-navigator-utah/Simulation/Models/Movies/'+parameter[:-4]+'_Images/'+parameter[:-4]+'_2_{0}.png'.format(str(i).zfill(3)))

#for i in range(8):
#    pos = (pos[0],pos[1]-1,pos[2])
#    focal = (focal[0],pos[1],focal[2])
#    camera.SetPosition(pos)
#    camera.SetFocalPoint(focal)
#    Render()
#    SaveScreenshot('/home/alex/pit-navigator-utah/Simulation/Models/Movies/'+parameter[:-4]+'_Images/'+parameter[:-4]+'_3_{0}.png'.format(str(i).zfill(3)))
sub = pos[2]/100*3.01
for i in range (28):
    pos = (pos[0],pos[1],pos[2]-sub)
    focal = (focal[0],focal[1],pos[2]-sub)
    camera.SetPosition(pos)
    camera.SetFocalPoint(focal)
    Render()
    SaveScreenshot('/home/alex/pit-navigator-utah/Simulation/Models/Movies/'+parameter[:-4]+'_Images/'+parameter[:-4]+'_4_{0}.png'.format(str(i).zfill(3)))
for i in range(8):
    camera.Pitch(3)
    Render()
    SaveScreenshot('/home/alex/pit-navigator-utah/Simulation/Models/Movies/'+parameter[:-4]+'_Images/'+parameter[:-4]+'_5_{0}.png'.format(str(i).zfill(3)))
camera.SetViewUp(0,0,1)
for i in range(120):
    camera.Pitch(66)
    camera.Yaw(3)
    camera.Pitch(-66)
    Render()
    SaveScreenshot('/home/alex/pit-navigator-utah/Simulation/Models/Movies/'+parameter[:-4]+'_Images/'+parameter[:-4]+'_6_{0}.png'.format(str(i).zfill(3)))