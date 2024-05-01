import matplotlib.pyplot as plt
import os
import numpy as np
import matplotlib
matplotlib.rcParams['font.serif'] = 'Palatino Linotype'
matplotlib.rcParams['font.family'] = 'serif'
matplotlib.rcParams['text.usetex'] = True
plt.rcParams.update({'font.size': 15})
matplotlib.rc('xtick', labelsize=15)
matplotlib.rc('ytick', labelsize=15)
matplotlib.rc('axes', labelsize=20)

matplotlib.rc('legend', fontsize=12)
matplotlib.rcParams['axes.titlesize'] = 20

file = (os.path.basename(__file__).replace(".py",""))
filepath = os.path.dirname(__file__) + "/"

x_data = [1,2,3.4,5,5.2,6,8,9]
y_data = [3,1,2,0.3,1.2,1.5,0.5,0.1]

# Create labels for each datapoint


# make a stemplot

plt.figure(figsize=(8,3))
plt.stem(x_data, y_data)
plt.xlabel("t")
plt.ylabel("|h(t)|")
plt.text(x_data[0]+0.1, y_data[0], "$\\alpha_0\mathrm{e}^{j\\theta_0}$")
plt.text(x_data[1]+0.1, y_data[1], "$\\alpha_1\mathrm{e}^{j\\theta_1}$")
plt.text(x_data[-1]+0.1, y_data[-1], "$\\alpha_k\mathrm{e}^{j\\theta_k}$")
plt.text(x_data[0]-0.05,0-0.2,"$\\tau_0$")
plt.text(x_data[1]-0.05,0-0.2,"$\\tau_1$")
plt.text(x_data[-1]-0.05,0-0.2,"$\\tau_k$")
plt.text(np.mean(x_data),np.min(y_data)-0.5,r"$t$")
plt.text(x_data[0]-0.5,np.mean(y_data),r"$|h(t)|$",rotation=90)

plt.xlim(0,10)
plt.ylim(-0.5,3.5)
plt.xticks([])
plt.yticks([])
plt.axis('off')



plt.savefig(filepath + file + ".pdf", format="pdf", transparent=True, bbox_inches='tight', pad_inches=0)


