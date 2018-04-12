import pandas as pd 
import matplotlib.pyplot as plt 
import os
import seaborn

data_dir = './simulation_data'
data = []
for f in os.listdir(data_dir):
    data.append(pd.read_csv(os.path.join(data_dir,f)))
    print('Loaded data log:'+f)

series =[]
index = []
for i,df in enumerate(data):
    blocks = df['Blocks'][1]
    radius = df['Radius'][1]
    f = len(df[df['Deadend']==True])
    s = len(df[df['Deadend']==False])
    tot = s+f
    s_frac = round(s/tot,3)
    f_frac = round(f/tot,3)
    series.append([s_frac,f_frac])
    index.append('r{}_b{}'.format(radius,blocks))

    


data = pd.DataFrame(series,columns=['Escaped','Deadend'],index = index)
print(data.head())

plt.figure()
data.plot(kind='bar')
plt.xlabel('Outcome')
plt.ylabel('Probability')
plt.title('Simulation Outcomes: r = Labyrinth radius, b = Number of Obstacles')

plt.show()



