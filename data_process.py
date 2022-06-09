import pandas as pd

tmp = pd.read_csv('./benchmark/random-32-32-20-result/plot_1_1_1.csv')
name = tmp.columns
df = pd.DataFrame({})
for i in name:
        df[i] = []

for i in range(1,26):
    tmp = pd.read_csv('./benchmark/random-32-32-20-result/plot_{}_0_0.csv'.format(i))
    data = tmp.loc[0]
    df.loc[i] = data

df.to_csv("./benchmark/random-32-32-20-result/plot_no_no_10_agent_real.csv", index=False, sep=',')