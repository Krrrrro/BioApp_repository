import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error
import joblib

# 가상의 데이터 생성 (실제 데이터로 대체 필요)
np.random.seed(0)
areas = np.random.rand(100, 1) * 10  # 0~10 제곱미터의 면적
weights = 2 * areas + 1 + 0.1 * np.random.randn(100, 1)  # 무게 = 2*면적 + 1 + 노이즈

# 데이터 분할 (학습 80%, 테스트 20%)
X_train, X_test, y_train, y_test = train_test_split(areas, weights, test_size=0.2, random_state=42)

# 모델 학습
model = LinearRegression()
model.fit(X_train, y_train)

# 모델 테스트
y_pred = model.predict(X_test)
mse = mean_squared_error(y_test, y_pred)
print(f"테스트 세트의 평균 제곱 오차: {mse}")

# 모델 저장
joblib.dump(model, 'weight_predictor.joblib')