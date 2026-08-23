import numpy as np
import plotly.graph_objects as go
import pandas as pd
import plotly.express as px

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
	CSV_FILE = 'force_data.csv'
	FORCE_THRESHOLD = 4.0  # Threshold X in Newtons (e.g., 0.5 N)

	# 1. Load raw CSV data
	df = pd.read_csv(CSV_FILE)

	# 2. Apply filtering options (choose the behavior that fits your analysis)

	# Option A: Deadband Clamping (Sets sensor chatter between -X and +X to 0.0 N)
	df['force_z_filtered'] = df['force_z'].apply(
	    lambda f: 0.0 if abs(f) < FORCE_THRESHOLD else f
	)

	# Option B: Masking (Converts noise to NaN, creating visible line gaps in Plotly)
	# df['force_z_filtered'] = df['force_z'].mask(df['force_z'].abs() < FORCE_THRESHOLD)

	# Option C: Hard Drop (Removes noise rows entirely)
	# df_filtered = df[df['force_z'].abs() >= FORCE_THRESHOLD]

	# 3. Plot Raw vs. Filtered Z-Force
	fig = go.Figure()

	# Raw trace for reference
	fig.add_trace(
	    go.Scatter(
		x=df['time_s'],
		y=df['force_z'],
		mode='lines',
		name='Raw Signal',
		line=dict(color='lightgray', width=1),
		opacity=0.7
	    )
	)

	# Filtered trace overlay
	fig.add_trace(
	    go.Scatter(
		x=df['time_s'],
		y=df['force_z_filtered'],
		mode='lines',
		name=f'Filtered (|Fz| >= {FORCE_THRESHOLD} N)',
		line=dict(color='royalblue', width=2)
	    )
	)

	fig.update_layout(
	    title='Cartesian Z-Force Analysis (Raw vs. Post-Filtered)',
	    xaxis_title='Time (s)',
	    yaxis_title='Z-Force (N)',
	    template='plotly_white',
	    hovermode='x unified'
	)

	fig.show()
	
plot_force()
