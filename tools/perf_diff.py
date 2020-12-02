import os
import pandas as pd
import matplotlib.pyplot as plt

pd.set_option('display.max_rows', 100)

def load(file):
    df = pd.read_csv(file + '.csv')
    df['Run'] = file
    return df

def generate_perf(file):
    os.system("echo 'Test,Suite,# Vertices,# Triangles,Runtime us,Iterations,Cost' > " + file + ".csv; ./raptor_mesh_decimation_regression_tests | grep PERF | perl -pe 's/.*PERF \\d - [#\\w\\s]+:\\s//g' | paste -d',' - - - - - - | sort | perl -pe 's/_((vertex)|(cumulative)|(merge))/,\\1/' >> " + file + ".csv")
    return load(file);

def averages(df, suite) :
    return df[df['Suite'].str.contains(suite)].groupby('Suite').mean().sort_values('Cost')

def compare_all(before, after, suite, diff):
    comp = before[before['Suite'] == suite].set_index('Test').join(after[after['Suite'] == suite].set_index('Test'), on='Test', lsuffix='l', rsuffix='r').reset_index()
    comp[diff] = (comp[diff + 'r'] - comp[diff + 'l']) / comp[diff + 'l']
    comp.set_index('Test')[diff].plot.bar()
    plt.show()
    return comp

def compare(before, after, diff):
    comp = before.groupby('Suite').mean().join(after.groupby('Suite').mean(), on='Suite', lsuffix='l', rsuffix='r').reset_index()
    comp[diff] = (comp[diff + 'r'] - comp[diff + 'l']) / comp[diff + 'l']
    comp.set_index('Suite')[diff].plot.bar()
    plt.show()
    return comp

def progress(dfs, col, stack) :
    df = pd.concat(dfs)
    df = df[['Run', 'Suite', col]].groupby(['Run', 'Suite']).mean().reset_index()
    df.pivot(index='Run', columns='Suite', values=col).plot.bar(stacked=stack)
    plt.show()
    return df

def progress_max(col, stack, filt) :
    if filt == '' :
        return progress([max_040, max_050, max_060, max_070, max_080, max_090, max_100], col, stack)
    else :
        return progress([
            max_040[max_040['Suite'].str.contains(filt)], 
            max_050[max_050['Suite'].str.contains(filt)], 
            max_060[max_060['Suite'].str.contains(filt)], 
            max_070[max_070['Suite'].str.contains(filt)], 
            max_080[max_080['Suite'].str.contains(filt)], 
            max_090[max_090['Suite'].str.contains(filt)], 
            max_100[max_100['Suite'].str.contains(filt)]], col, stack)

def progress_stride(col, stack, filt) :
    if filt == '' :
        return progress([stride_01, stride_02, stride_05, stride_10, stride_15, stride_20, stride_50], col, stack)
    else:
        return progress([
            stride_01[stride_01['Suite'].str.contains(filt)], 
            stride_02[stride_02['Suite'].str.contains(filt)], 
            stride_05[stride_05['Suite'].str.contains(filt)], 
            stride_10[stride_10['Suite'].str.contains(filt)], 
            stride_15[stride_15['Suite'].str.contains(filt)], 
            stride_20[stride_20['Suite'].str.contains(filt)], 
            stride_50[stride_50['Suite'].str.contains(filt)]], col, stack)

def progress_clean(col, stack) :
    return progress([clean_10, clean_15, clean_20, clean_25, clean_30, clean_35, clean_50, clean_80 ], col, stack)
