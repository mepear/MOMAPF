import pandas as pd

tmp = pd.read_csv('./benchmark/empty-16-16-result/plot_1_1.csv')
name = tmp.columns
df = pd.DataFrame({})
for i in name:
        df[i] = []

for i in range(1,26):
    tmp = pd.read_csv('./benchmark/empty-16-16-result/plot_{}_1.csv'.format(i))
    data = tmp.loc[0]
    df.loc[i] = data

df.to_csv("./benchmark/empty-16-16-result/plot_has_12_agent_new.csv", index=False, sep=',')