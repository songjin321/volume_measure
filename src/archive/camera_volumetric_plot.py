from plotly.offline import plot
import plotly.graph_objs as go
import numpy as np

plot_data = np.load('plot.npy')
cam_R = plot_data[()]['cam_R']
cam_t = plot_data[()]['cam_t']
verts = plot_data[()]['verts']
faces = plot_data[()]['faces']
voxels_value_flatten = plot_data[()]['voxel_value']
object_point_3D = plot_data[()]['object_point']

trace_object = go.Mesh3d(x=verts[:,0],y=verts[:,1],z=verts[:,2],i=faces[:,0],j=faces[:,1],k=faces[:,2])
trace_camera = go.Cone()
#trace_cube = go.Volume(x=,y=object_point_3D[1,:],z=object_point_3D[2,:],value=voxels_value_flatten)

# try to using plotly to imshow 3d mesh and camera pose
x,y,z=zip(*verts)
trace_point = go.Scatter3d(x=x,y=y,z=z,mode='markers',
    marker=dict(
        color='rgb(0, 0, 255)',
        size=2,
        symbol='circle',
        line=dict(
            color='rgb(204, 204, 204)',
            width=1
        ),
        opacity=0.9
    ))
plot([trace_point, trace_object])
print('fdsa')