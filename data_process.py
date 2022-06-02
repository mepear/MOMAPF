import pandas as pd

tmp = pd.read_csv('./benchmark/room-32-32-4-result/plot_1_1_1.csv')
name = tmp.columns
df = pd.DataFrame({})
for i in name:
        df[i] = []

for i in range(1,26):
    tmp = pd.read_csv('./benchmark/room-32-32-4-result/plot_{}_1_0.csv'.format(i))
    data = tmp.loc[0]
    df.loc[i] = data

df.to_csv("./benchmark/room-32-32-4-result/plot_has_no_8_agent.csv", index=False, sep=',')