import pandas as pd

csv_test = pd.read_csv('/home/retta/catkin_ws/src/Kimdom/rep/mando_map_final.csv')
print(csv_test)
df = pd.DataFrame(csv_test, columns = ['x', 'y', 'road', 'event'])

list_x = df['x'].values
list_y = df['y'].values
df['road'] = 0
df['event'] = 'run'

for i in range(len(list_x)-1):
    if(abs(list_x[i] - list_x[i+1]) > 0.1):
        if(abs(list_y[i] - list_y[i+1]) > 0.1):
            df.loc[i,'road'] = 'curve'
        else:
            df.loc[i,'road'] = 'straight'
    elif(abs(list_y[i]-list_y[i+1]) > 0.1):
        df.loc[i,'road'] = 'curve'
    else:
        df.loc[i,'road'] = 'stop'

for i in range(len(list_x)-1):
    if ((2859<=i<=2874) | (5035<=i<=5042) | (8552<=i<=8562)):
        df.loc[i, 'event'] = 'TrafficLight'
    if ((9931<=i<=10072)):
        df.loc[i, 'event'] = 'accel>=20'


df.to_csv("/home/retta/catkin_ws/src/mando_morai_test/rep/mando_map_final_marker.csv", index = False)
print(df)