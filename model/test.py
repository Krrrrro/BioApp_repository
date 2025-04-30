import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error
import joblib

loaded_model = joblib.load('weight_predictor.joblib')

def predict_weight(area):
    """
    주어진 면적에 대한 무게를 예측합니다.
    
    Parameters:
    area (float): 제곱미터 단위의 면적
    
    Returns:
    float: 예측된 무게 (킬로그램)
    """
    return loaded_model.predict([[area]])[0][0]

# 예시: 면적 5 제곱미터에 대한 무게 예측
# predicted_weight = predict_weight(5)
# print(f"면적 5 제곱미터에 대한 예측 무게: {predicted_weight} kg")