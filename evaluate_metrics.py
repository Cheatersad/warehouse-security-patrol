import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import accuracy_score, precision_score, recall_score, confusion_matrix, roc_curve, auc

# 1. Generate Synthetic Data
# Seed for reproducibility
np.random.seed(42)
n_normal = 800
n_abnormal = 200

# Normal errors: Low mean, low variance
normal_errors = np.random.normal(loc=0.1, scale=0.04, size=n_normal)
normal_errors = np.abs(normal_errors) 

# Abnormal errors: High mean, higher variance
abnormal_errors = np.random.normal(loc=0.55, scale=0.18, size=n_abnormal)
abnormal_errors = np.abs(abnormal_errors)

reconstruction_errors = np.concatenate([normal_errors, abnormal_errors])
true_labels = np.concatenate([np.zeros(n_normal), np.ones(n_abnormal)])

# 2. Calculate Threshold (95th percentile of normal reconstruction errors)
threshold = np.percentile(normal_errors, 95)

# 3. Plot Histogram
plt.figure(figsize=(10, 6))
plt.hist(normal_errors, bins=50, color='blue', alpha=0.5, label='Normal Samples', density=True)
plt.hist(abnormal_errors, bins=50, color='red', alpha=0.5, label='Abnormal Samples', density=True)
plt.axvline(threshold, color='black', linestyle='--', linewidth=2, label=f'Anomaly Threshold ({threshold:.3f})')
plt.title('Reconstruction Error Distribution (Normal vs Abnormal)')
plt.xlabel('Reconstruction Error (MSE)')
plt.ylabel('Probability Density')
plt.legend()
plt.grid(axis='y', alpha=0.3)
plt.savefig('reconstruction_histogram.png')
plt.close()

# 4. Calculate Performance Metrics
# Classify as anomaly (1) if error > threshold
predictions = (reconstruction_errors > threshold).astype(int)

accuracy = accuracy_score(true_labels, predictions)
precision = precision_score(true_labels, predictions)
recall = recall_score(true_labels, predictions)
cm = confusion_matrix(true_labels, predictions)

print("-" * 45)
print("   ANOMALY DETECTION EVALUATION REPORT")
print("-" * 45)
print(f"Optimal Threshold (95% Normal): {threshold:.4f}")
print(f"Accuracy:  {accuracy:.4f}")
print(f"Precision: {precision:.4f}")
print(f"Recall:    {recall:.4f}")
print("-" * 45)
print("CONFUSION MATRIX:")
print(f"True Negatives (Normal):    {cm[0,0]}")
print(f"False Positives (False Alarm): {cm[0,1]}")
print(f"False Negatives (Missed):    {cm[1,0]}")
print(f"True Positives (Detected):   {cm[1,1]}")
print("-" * 45)

# 5. Plot ROC Curve
fpr, tpr, _ = roc_curve(true_labels, reconstruction_errors)
roc_auc = auc(fpr, tpr)

plt.figure(figsize=(10, 6))
plt.plot(fpr, tpr, color='darkorange', lw=2, label=f'ROC curve (AUC = {roc_auc:.4f})')
plt.plot([0, 1], [0, 1], color='navy', lw=2, linestyle='--')
plt.xlim([0.0, 1.0])
plt.ylim([0.0, 1.05])
plt.xlabel('False Positive Rate')
plt.ylabel('True Positive Rate')
plt.title('Receiver Operating Characteristic (ROC)')
plt.legend(loc="lower right")
plt.grid(alpha=0.3)
plt.savefig('roc_curve.png')
plt.close()

print(f"Evaluation complete.")
print(f"Plots saved: reconstruction_histogram.png, roc_curve.png")
print("-" * 45)
