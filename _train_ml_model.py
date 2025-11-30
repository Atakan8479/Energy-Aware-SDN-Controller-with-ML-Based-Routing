#!/usr/bin/env python3
"""
ML Training Script for SDN Smart Routing
Uses the dataset collected by SDN Controller to train advanced ML models
"""

import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.ensemble import RandomForestClassifier
from sklearn.neighbors import KNeighborsClassifier
from sklearn.tree import DecisionTreeClassifier
from sklearn.svm import SVC
from sklearn.metrics import accuracy_score, classification_report, confusion_matrix
import matplotlib.pyplot as plt
import seaborn as sns
import joblib

class SDNMLTrainer:
    def __init__(self, dataset_path='sdn_dataset.csv'):
        self.dataset_path = dataset_path
        self.models = {}
        self.scaler = StandardScaler()
        self.best_model = None
        self.best_accuracy = 0.0
        
    def load_data(self):
        """Load and preprocess the SDN dataset"""
        print("Loading dataset from:", self.dataset_path)
        
        try:
            df = pd.read_csv(self.dataset_path)
            print(f"Dataset loaded: {len(df)} samples")
            print("\nDataset info:")
            print(df.info())
            print("\nFirst few rows:")
            print(df.head())
            
            return df
        except Exception as e:
            print(f"Error loading dataset: {e}")
            return None
    
    def preprocess_data(self, df):
        """Preprocess the dataset"""
        print("\nPreprocessing data...")
        
        # Feature columns
        feature_cols = ['src_addr', 'dest_addr', 'src_battery', 'dest_battery', 
                       'path_distance', 'path_delay', 'path_quality']
        
        # Target column
        target_col = 'chosen_path'
        
        # Extract features and target
        X = df[feature_cols]
        y = df[target_col]
        
        print(f"Features shape: {X.shape}")
        print(f"Target shape: {y.shape}")
        print(f"Number of unique paths: {y.nunique()}")
        
        # Split data
        X_train, X_test, y_train, y_test = train_test_split(
            X, y, test_size=0.2, random_state=42, stratify=y
        )
        
        # Scale features
        X_train_scaled = self.scaler.fit_transform(X_train)
        X_test_scaled = self.scaler.transform(X_test)
        
        return X_train_scaled, X_test_scaled, y_train, y_test, X_train, X_test
    
    def train_models(self, X_train, X_test, y_train, y_test):
        """Train multiple ML models"""
        print("\n" + "="*50)
        print("TRAINING ML MODELS")
        print("="*50)
        
        # Define models
        models = {
            'Random Forest': RandomForestClassifier(n_estimators=100, random_state=42),
            'K-Nearest Neighbors': KNeighborsClassifier(n_neighbors=5),
            'Decision Tree': DecisionTreeClassifier(max_depth=10, random_state=42),
            'SVM': SVC(kernel='rbf', random_state=42)
        }
        
        results = {}
        
        for name, model in models.items():
            print(f"\n--- Training {name} ---")
            
            # Train
            model.fit(X_train, y_train)
            
            # Predict
            y_pred = model.predict(X_test)
            
            # Evaluate
            accuracy = accuracy_score(y_test, y_pred)
            results[name] = {
                'model': model,
                'accuracy': accuracy,
                'predictions': y_pred
            }
            
            print(f"Accuracy: {accuracy:.4f}")
            
            # Check if this is the best model
            if accuracy > self.best_accuracy:
                self.best_accuracy = accuracy
                self.best_model = model
                self.best_model_name = name
        
        self.models = results
        return results
    
    def evaluate_models(self, y_test):
        """Evaluate and visualize model performance"""
        print("\n" + "="*50)
        print("MODEL EVALUATION")
        print("="*50)
        
        # Plot accuracy comparison
        plt.figure(figsize=(10, 6))
        model_names = list(self.models.keys())
        accuracies = [self.models[name]['accuracy'] for name in model_names]
        
        plt.bar(model_names, accuracies, color=['blue', 'green', 'orange', 'red'])
        plt.xlabel('Model')
        plt.ylabel('Accuracy')
        plt.title('Model Performance Comparison')
        plt.ylim([0, 1.0])
        plt.xticks(rotation=45, ha='right')
        
        for i, acc in enumerate(accuracies):
            plt.text(i, acc + 0.02, f'{acc:.3f}', ha='center')
        
        plt.tight_layout()
        plt.savefig('model_comparison.png')
        print("Saved: model_comparison.png")
        
        # Detailed report for best model
        print(f"\n--- Best Model: {self.best_model_name} ---")
        print(f"Accuracy: {self.best_accuracy:.4f}")
        
        y_pred_best = self.models[self.best_model_name]['predictions']
        
        print("\nClassification Report:")
        print(classification_report(y_test, y_pred_best))
        
        # Confusion matrix
        cm = confusion_matrix(y_test, y_pred_best)
        plt.figure(figsize=(8, 6))
        sns.heatmap(cm, annot=True, fmt='d', cmap='Blues')
        plt.xlabel('Predicted Path')
        plt.ylabel('Actual Path')
        plt.title(f'Confusion Matrix - {self.best_model_name}')
        plt.tight_layout()
        plt.savefig('confusion_matrix.png')
        print("Saved: confusion_matrix.png")
    
    def feature_importance(self, X_train_original):
        """Analyze feature importance (for tree-based models)"""
        if self.best_model_name in ['Random Forest', 'Decision Tree']:
            print("\n" + "="*50)
            print("FEATURE IMPORTANCE")
            print("="*50)
            
            feature_names = X_train_original.columns
            importances = self.best_model.feature_importances_
            
            # Sort by importance
            indices = np.argsort(importances)[::-1]
            
            print("\nFeature ranking:")
            for i, idx in enumerate(indices):
                print(f"{i+1}. {feature_names[idx]}: {importances[idx]:.4f}")
            
            # Plot
            plt.figure(figsize=(10, 6))
            plt.bar(range(len(importances)), importances[indices])
            plt.xticks(range(len(importances)), 
                      [feature_names[i] for i in indices], 
                      rotation=45, ha='right')
            plt.xlabel('Feature')
            plt.ylabel('Importance')
            plt.title('Feature Importance')
            plt.tight_layout()
            plt.savefig('feature_importance.png')
            print("Saved: feature_importance.png")
    
    def save_model(self, filename='sdn_best_model.pkl'):
        """Save the best model"""
        if self.best_model:
            joblib.dump({
                'model': self.best_model,
                'scaler': self.scaler,
                'model_name': self.best_model_name,
                'accuracy': self.best_accuracy
            }, filename)
            print(f"\nModel saved: {filename}")
    
    def load_model(self, filename='sdn_best_model.pkl'):
        """Load a saved model"""
        data = joblib.load(filename)
        self.best_model = data['model']
        self.scaler = data['scaler']
        self.best_model_name = data['model_name']
        self.best_accuracy = data['accuracy']
        print(f"Model loaded: {self.best_model_name} (Accuracy: {self.best_accuracy:.4f})")
    
    def predict_path(self, src_addr, dest_addr, src_battery, dest_battery, 
                     path_distance, path_delay, path_quality):
        """Predict best path for new flow"""
        if not self.best_model:
            print("Error: No trained model available")
            return None
        
        # Prepare input
        X_new = np.array([[src_addr, dest_addr, src_battery, dest_battery, 
                          path_distance, path_delay, path_quality]])
        X_new_scaled = self.scaler.transform(X_new)
        
        # Predict
        predicted_path = self.best_model.predict(X_new_scaled)[0]
        
        return predicted_path

def main():
    """Main training pipeline"""
    print("="*60)
    print("SDN SMART ROUTING - ML TRAINING")
    print("="*60)
    
    # Initialize trainer
    trainer = SDNMLTrainer('sdn_dataset.csv')
    
    # Load data
    df = trainer.load_data()
    if df is None or len(df) < 10:
        print("\nInsufficient data for training!")
        print("Please run the simulation longer to collect more samples.")
        return
    
    # Preprocess
    X_train, X_test, y_train, y_test, X_train_orig, X_test_orig = trainer.preprocess_data(df)
    
    # Train models
    results = trainer.train_models(X_train, X_test, y_train, y_test)
    
    # Evaluate
    trainer.evaluate_models(y_test)
    
    # Feature importance
    trainer.feature_importance(X_train_orig)
    
    # Save best model
    trainer.save_model()
    
    print("\n" + "="*60)
    print("TRAINING COMPLETED!")
    print("="*60)
    print(f"Best Model: {trainer.best_model_name}")
    print(f"Accuracy: {trainer.best_accuracy:.4f}")
    print("\nGenerated files:")
    print("  - sdn_best_model.pkl")
    print("  - model_comparison.png")
    print("  - confusion_matrix.png")
    print("  - feature_importance.png")
    
    # Example prediction
    print("\n" + "="*60)
    print("EXAMPLE PREDICTION")
    print("="*60)
    predicted_path = trainer.predict_path(
        src_addr=1, dest_addr=3, 
        src_battery=85.0, dest_battery=90.0,
        path_distance=50.0, path_delay=0.05, path_quality=75.0
    )
    print(f"Predicted optimal path: {predicted_path}")

if __name__ == '__main__':
    main()
