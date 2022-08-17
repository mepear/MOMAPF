import pandas as pd

tmp = pd.read_csv('./benchmark/room-32-32-4-result/plot_5_1_0.csv')
name = tmp.columns
df = pd.DataFrame({})
for i in name:
        df[i] = []

for l in range(4):
    for num in range(2, 11):
        for i in range(1,26):
            try:
                tmp = pd.read_csv('./benchmark/room-32-32-4-result/plot_{}_{}_{}.csv'.format(num, i, l))
                data = tmp.loc[0]
                df.loc[i] = data
            except:
                df.loc[i] = [0, None, None, None, None]
        df.to_csv("./benchmark/room-32-32-4-result-set/random3_{}_{}.csv".format(num, l), index=False, sep=',')
        df = df.drop(index=df.index)
