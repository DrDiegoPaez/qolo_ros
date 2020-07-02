from os.path import expanduser
import math
from bokeh.plotting import figure, output_file, show

all_filenames = [
"v0.500000-d0.200000.txt",
"v0.500000-d0.600000.txt",
"v0.500000-d1.000000.txt",
"v0.700000-d0.200000.txt",
"v0.700000-d0.600000.txt",
"v0.700000-d1.000000.txt",
"v0.900000-d0.200000.txt",
"v0.900000-d0.600000.txt",
"v1.200000-d0.200000.txt",
"v1.200000-d0.600000.txt",
"v1.200000-d1.000000.txt",
"v1.500000-d0.200000.txt",
"v1.500000-d0.600000.txt",
"v1.500000-d1.000000.txt"
]

def compute_mean_and_std(filename):
	with open(expanduser('~/' + filename), 'r') as f:
	    vt_list = [float(word) for line in f for word in line.split()]

	    v_list = vt_list[0:len(vt_list)/2]
	    t_list = vt_list[len(vt_list)/2:]

	    #v_list = v_list[len(v_list)/5:]
	    #t_list = t_list[len(t_list)/5:]

	    dtsum = 0.0
	    t = []
	    for dt in t_list:
	    	t.append(dtsum)
	    	dtsum += dt

	    samples = []
	    n = 0
	    local_mean = 0.0
	    for i in range(len(t)):
	    	if t[i] < dtsum/10*(len(samples)+0.5):
	    		continue
	    	n += 1
	    	local_mean += v_list[i]

	    	if (t[i] > dtsum/10*(len(samples)+1.0)) or ((i == len(t)-1) and (len(samples) < 10)):
	    		samples.append(local_mean/n)
	    		local_mean = 0.0
	    		n = 0

	    mean_mean = 0.0
	    for m in samples:
	    	mean_mean += m
	    mean_mean /= 10

	    std = 0.0
	    for m in samples:
	    	std += (m - mean_mean)*(m - mean_mean)
	    std = math.sqrt(std/9)

	    return (mean_mean, std)

	    #N = len(v_list)
	    #mean = 0.0
	    #for i in range(N):
	    #    mean += v_list[i]*t_list[i]/dtsum

	    #std = 0.0
	    #for i in range(N):
	    #    std += (v_list[i] - mean)*(v_list[i] - mean)*t_list[i]
	    #std = math.sqrt(std/(dtsum - 1.0))

	    print ("mean = %f" % mean_mean)
	    print ("std = %f" % std)

	    # output to static HTML file
	    output_file("lines.html")

	    # create a new plot with a title and axis labels
	    p = figure(title="velocity samples", x_axis_label='sample_index', y_axis_label='v')

	    # add a line renderer with legend and line thickness
	    p.line(range(10), samples, legend_label="velocity", line_width=2)

	    # show the results
	    show(p)

for f in all_filenames: