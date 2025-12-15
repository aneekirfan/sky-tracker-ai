# 🛡️ Proactive Ransomware Detection Using AI

![Python](https://img.shields.io/badge/Python-3.8%2B-blue?style=for-the-badge&logo=python)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Maintained-orange?style=for-the-badge)
![ML](https://img.shields.io/badge/AI-Random%20Forest-purple?style=for-the-badge)

> **Safeguarding Indian MSMEs Against Evolving Cyber Threats.**

---

## 📖 Overview

In the rapidly shifting sands of the digital economy, Micro, Small, and Medium Enterprises (MSMEs) are becoming the bedrock of India's GDP, contributing nearly 30%[cite: 12]. [cite_start]However, this digital integration is a double-edged sword; it has rendered these businesses susceptible to a myriad of cyber threats, with **Ransomware** standing out as the most financially destructive[cite: 20].

This project presents a robust, budget-friendly cybersecurity solution designed to detect ransomware proactively. [cite_start]Leveraging the power of **Supervised Machine Learning** (specifically Random Forest Classifiers), this tool analyzes file behavior, system interactions, and memory usage to identify threats before encryption occurs[cite: 38, 39].

---

## 🚀 Key Features

* [cite_start]**High Accuracy:** The model boasts an exceptional accuracy rate of **99.76%** with minimized false positives[cite: 122, 123].
* [cite_start]**Imbalance Handling:** Utilizes **SMOTE** (Synthetic Minority Oversampling Technique) to ensure the model doesn't bias towards the majority class, creating a fair fight between benign and malicious samples[cite: 103].
* [cite_start]**Feature Optimization:** Culls 77 initial features down to the 58 most critical indicators, focusing on registry writes, file entropy, and process monitoring[cite: 99, 100].
* [cite_start]**Scalable & Lightweight:** Designed to operate efficiently on limited system resources (Intel i7, 8GB RAM tested), making it ideal for the MSME sector[cite: 92].

---

## 🛠️ Methodology & Architecture

The system follows a rigorous pipeline—from raw data ingestion to serialized model deployment.

### 1. Data Preprocessing
We handle the messy reality of raw datasets by cleaning missing values and converting hexadecimal strings to integers. [cite_start]Categorical features undergo **Label Encoding** to become machine-readable[cite: 97].

### 2. Class Balancing
Ransomware is a rare event in the wild. [cite_start]To prepare the model for this, we balance the dataset using SMOTE, generating synthetic examples of ransomware to ensure the classifier learns the nuances of the minority class effectively[cite: 103].

### 3. Model Training
[cite_start]We employ a **Random Forest Classifier**[cite: 85]. Why? [cite_start]Because it excels at capturing non-linear relationships in data and resists overfitting, which is crucial when dealing with the high-stakes nature of malware detection[cite: 109].

### 4. Evaluation Metrics
We don't just guess; we measure. [cite_start]The model is evaluated using standard metrics[cite: 116]:
* **Accuracy**
* **Precision**
* **Recall**
* **F1-Score**

---

## 📊 Performance Results

The model was put through its paces, and the results are frankly quite telling.

| Metric | Score |
| :--- | :--- |
| **Accuracy** | 99.76% |
| **Precision** | 0.997 |
| **Recall** | 0.998 |
| **F1-Score** | 0.997 |

[cite_start]*Performance metrics derived from experimental results[cite: 122].*

[cite_start]As seen in the confusion matrix analysis, the system successfully identified 1428 ransomware samples with only negligible misclassifications[cite: 124].

---

## 💻 Installation & Usage

### Prerequisites
Ensure you have Python installed. The system relies on standard ML libraries.

```bash
pip install pandas scikit-learn matplotlib seaborn imbalanced-learn joblib
```

## 🧑‍🏭 Running the Training Module

### To train the model from scratch using the dataset:
```bash
python main.py
```
This will preprocess the data, apply SMOTE, train the Random Forest, and serialize the model to ransomware_detector_model.pkl.

## 🔮 Predicting on New Data

### The script is configured to load the saved model and run predictions on input_features.csv.
```bash
# Snippet from main.py
loaded_model = load_model("ransomware_detector_model.pkl")
predictions_df = predict_from_csv(loaded_model, input_csv_path)
```


## ⌛ Future scope

While the aggregated random forest model is showing some serious promise in detecting ransomware, we aren't done yet; several future research avenues remain open. We could, for instance, integrate the model with bleeding-edge stuff like deep learning or reinforcement learning to boost adaptability against those nasty, emerging ransomware variants. Also, expanding the dataset to snag post-2023 samples would definitely ensure the thing stays relevant. Real-time deployment in wild, diverse environments? That offers a massive chance to fine-tune performance for actual, practical challenges. Plus, testing adversarial techniques could really toughen up the model’s resilience against evasion attempts. And hey, applying this approach to other malware types could broaden its utility, extending its impact across the cybersecurity landscape.

## 📜 Citations & References

This project is based on the research "Proactive Ransomware Detection Using AI: Safeguarding Indian MSMEs Against Evolving Cyber Threats".

Indian Computer Emergency Response Team (CERT-In). Annual report 2022.

S. I. Bae et al., "Ransomware detection using machine learning algorithms," 2020.

B. M. Khammas, "Ransomware detection using random forest technique," 2020.

## Author: Mohammad Aneek Khan
## Institution: Central University of Kashmir
