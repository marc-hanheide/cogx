f = file("data/accuracy_NBest5_SeSyAC_R_b120_TTT.txt", 'r')
text = f.read()
TP_EM = float(text.count("exact-match result: TP"))
FP_EM = float(text.count("exact-match result: FP"))
FN_EM = float(text.count("exact-match result: FN"))
precision_EM = TP_EM / (TP_EM+FP_EM)
recall_EM = TP_EM / (TP_EM+FN_EM)
f_1_EM = 2*precision_EM*recall_EM / (precision_EM+recall_EM)

TP_EM = float(text.count("exact-match result: TP"))
FP_EM = float(text.count("exact-match result: FP"))
FN_EM = float(text.count("exact-match result: FN"))
precision_EM = TP_EM / (TP_EM+FP_EM)
recall_EM = TP_EM / (TP_EM+FN_EM)
f_1_EM = 2*precision_EM*recall_EM / (precision_EM+recall_EM)


TP_PM = 0.0
FP_PM = 0.0
FN_PM =0.0
TN_PM = 0.0
for l in text.split("\n"):
	split1 = l.split(" TP & ")
	if len(split1) == 2:
		split2 = split1[0].split("(")
		TP_PM = TP_PM + int(split2[2]) 
		split3=split1[1].split(" FP")
		FP_PM = FP_PM + abs(int(split3[0]))

	split1 = l.split(" TN & ")
	if len(split1) == 2:
		split2 = split1[0].split("(")
		TN_PM = TP_PM + int(split2[2]) 
		split3=split1[1].split(" FN")
		FN_PM = FN_PM + abs(int(split3[0]))

precision_PM = TP_PM / (TP_PM+FP_PM)
recall_PM = TP_PM / (TP_PM+FN_PM)
f_1_PM = 2*precision_PM*recall_PM / (precision_PM+recall_PM)

print
print "EXACT MATCH:"
print "precision: %f"%precision_EM
print "recall: %f"%recall_EM
print "F_1: %f"%f_1_EM
print
print "PARTIAL MATCH:"
print "precision: %f"%precision_PM
print "recall: %f"%recall_PM
print "F_1: %f"%f_1_PM


