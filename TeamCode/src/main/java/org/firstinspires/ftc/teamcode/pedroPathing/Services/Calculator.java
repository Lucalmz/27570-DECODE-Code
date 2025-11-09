package org.firstinspires.ftc.teamcode.pedroPathing.Services;

import com.pedropathing.math.Vector;

public class Calculator {
    public static double DegreeToPitchServo(double degree){
        if(degree<54.9||degree>90){
            throw new IllegalArgumentException("Degree"+degree+"is not available");
        }
        return (90-degree)*0.02857142857142857142857142857143;
    }
    /**
            * 根据玩家输入和辅助瞄准修正，计算最终混合后的控制输出。
            * 该方法采用自适应混合权重，权重大小由辅助瞄准修正向量的长度决定。
            *
            * @param playerInput 玩家的输入向量。x为左右[-1,1]，y为前后[-1,1]。
            * @param assistVector 辅助瞄准程序计算出的修正向量。x为左右[-1,1]，y为前后[-1,1]。
            * @param masterAssistStrength 一个总体的辅助强度系数，用于缩放辅助效果 (通常 >= 0)。
            *                               设置为1.0意味着在最大修正时，权重为100%。
            *                               设置为1.2则会更快达到100%的权重。
            * @return 返回混合后的最终控制向量 (Vector2D)。
            */
    public static Vector calculateMixedControl(Vector playerInput, Vector assistVector, double masterAssistStrength) {

        // 计算辅助修正向量的长度 (Magnitude) ---
        double assistMagnitude = Math.sqrt(
                assistVector.getXComponent() * assistVector.getXComponent() + assistVector.getYComponent() * assistVector.getYComponent()
        );

        // 计算有效的混合权重 (Effective Strength) ---
        double rawStrength = assistMagnitude * masterAssistStrength;
        double effectiveStrength = Math.max(0.0, Math.min(1.0, rawStrength));

        // --- 步骤 3: 应用线性插值 (Lerp) 进行混合 ---
        // 公式: final = (1 - weight) * player + weight * assist
        double finalX = (1.0 - effectiveStrength) * playerInput.getXComponent() + effectiveStrength * assistVector.getXComponent();
        double finalY = (1.0 - effectiveStrength) * playerInput.getYComponent() + effectiveStrength * assistVector.getYComponent();

        return new Vector(finalX, finalY);
    }
}
