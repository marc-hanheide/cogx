from scipy import stats
res = stats.ttest_ind(WERResulstbase[:len(examples)], WERResults)
print "p-value: %f (should be < 0.05)"%res[1]
