import plotly.graph_objects as go
import numpy as np
from scipy.spatial.transform import Rotation as R

# Define a rotation for demonstration (e.g., 45 degrees around the Z-axis)
rotation = R.from_euler('z', 45, degrees=True)

# Standard unit vectors
i_hat = np.array([1, 0, 0])  # X-axis
j_hat = np.array([0, 1, 0])  # Y-axis
k_hat = np.array([0, 0, 1])  # Z-axis

# Rotate the unit vectors to get the new orientation
u_x = rotation.apply(i_hat)  # New X-axis after rotation
u_y = rotation.apply(j_hat)  # New Y-axis after rotation
u_z = rotation.apply(k_hat)  # New Z-axis after rotation

# Define vectors in dictionary for easier plotting
vectors = {
    "New X-axis (u_x)": {"start": [0, 0, 0], "end": u_x},
    "New Y-axis (u_y)": {"start": [0, 0, 0], "end": u_y},
    "New Z-axis (u_z)": {"start": [0, 0, 0], "end": u_z},
}

# Create the 3D plot
fig = go.Figure()

# Add each oriented unit vector as an arrow (quiver plot)
for name, vector in vectors.items():
    fig.add_trace(
        go.Scatter3d(
            x=[vector["start"][0], vector["end"][0]],
            y=[vector["start"][1], vector["end"][1]],
            z=[vector["start"][2], vector["end"][2]],
            marker=dict(size=2),
            line=dict(width=5),
            name=name
        )
    )

# Customize layout for visualization
fig.update_layout(
    scene=dict(
        xaxis=dict(nticks=4, range=[-1, 1.5], title="X-axis"),
        yaxis=dict(nticks=4, range=[-1, 1.5], title="Y-axis"),
        zaxis=dict(nticks=4, range=[-1, 1.5], title="Z-axis"),
        aspectratio=dict(x=1, y=1, z=1),
    ),
    title="Oriented Coordinate Frame in 3D Space"
)

fig.show()
