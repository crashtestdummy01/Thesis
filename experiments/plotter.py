import numpy as np
import plotly.graph_objects as go
import pandas as pd
import plotly.express as px
import sys

def plot_trajectory():
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

def plot_force():
	exp_num = sys.argv[1] if len(sys.argv) > 1 else "0"
	CSV_FILE = f'force_data-{exp_num}.csv'
	FORCE_THRESHOLD = 0.0  # Threshold X in Newtons (e.g., 0.5 N)

	# 1. Load raw CSV data
	df = pd.read_csv(CSV_FILE)
	
	df['force_mag'] = np.sqrt(df['force_x']**2 + df['force_y']**2 + df['force_z']**2)

	fig = go.Figure()

	# Filtered trace overlay
	fig.add_trace(
	    go.Scatter(
		x=df['time_s'],
		y=df['force_mag'],
		mode='lines',
		name=f'Force (N))',
		line=dict(color='royalblue', width=2)
	    )
	)

	fig.update_layout(
	    title='Cartesian contact Force',
	    xaxis_title='Time (s)',
	    yaxis_title='Z-Force (N)',
	    template='plotly_white',
	    hovermode='x unified'
	)

	fig.show()
	
plot_force()
