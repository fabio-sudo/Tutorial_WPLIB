package frc.robot.util;

/**
 * Física de um mecanismo que roda EM PARALELO com o real (padrão do
 * telescópio 9458). Cada XIOSim implementa: é a mesma física da simulação,
 * mas recebendo a tensão MEDIDA no motor em vez do pedido do controle.
 *
 * Unidades: as mesmas das entradas do IO do mecanismo (voltas do motor, RPM,
 * metros...). ParallelModel converte para o log.
 */
public interface MechanismModel {

    /** Copia o estado medido: o modelo parte de onde o real está. */
    void setModelState(double position, double velocity);

    /** Avança um ciclo do robô (20 ms) com esta tensão no motor. */
    void stepModel(double volts);

    /** Mecanismos de velocidade (rodas, rolos) devolvem 0. */
    double modelPosition();

    double modelVelocity();

    /** Corrente de UM motor, como a leitura do IO real. */
    double modelCurrentAmps();
}
