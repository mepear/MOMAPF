import pandas as pd

tmp = pd.read_csv('./benchmark/maze-32-32-2-result/plot_7_1_1_1.csv')
name = tmp.columns
df = pd.DataFrame({})
for i in name:
        df[i] = []

for num in range(7, 8):
    for i in range(1,26):
        tmp = pd.read_csv('./benchmark/maze-32-32-2-result/plot_{}_{}_0_0.csv'.format(num, i))
        data = tmp.loc[0]
        df.loc[i] = data
    df.to_csv("./benchmark/maze-32-32-2-result/real_{}_0.csv".format(num), index=False, sep=',')
    df = df.drop(index=df.index)
