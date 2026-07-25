import numpy as np
import plotly.graph_objects as go

data = np.load('long_trajectory.npy')
data[:, 0] += 0.2
data *= 1.2
x_coords = data[:, 0]
y_coords = data[:, 1]
z_coords = data[:, 2]


# 3. Create the 3D Scatter Plot
fig = go.Figure(
    data=[
        go.Scatter3d(
            x=x_coords,
            y=y_coords,
            z=z_coords,
            mode="markers",
            marker=dict(
                size=3,  # Small size works great for dense point clouds
                color=z_coords,  # Color points by their Z height
                colorscale="Viridis",  # A nice, smooth color palette
                opacity=0.8,
            ),
        )
    ]
)

# 4. Clean up the layout and aspect ratio
fig.update_layout(
    title="3D Airplane Point Cloud",
    scene=dict(
        xaxis_title="X",
        yaxis_title="Y",
        zaxis_title="Z",
        # 'data' matches physical proportions so your airplane doesn't look squished
        aspectmode="data",
    ),
    margin=dict(l=0, r=0, b=0, t=40),  # Tight margins to give the plot room
)

# 5. Render the plot in your browser or notebook
fig.show()
