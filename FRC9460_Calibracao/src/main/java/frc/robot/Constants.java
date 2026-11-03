// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * 🔎 LEGENDA DOS ÍCONES NOS COMENTÁRIOS (Ctrl+Shift+F com o ícone acha tudo):
 *   🛑 SEGURANÇA  o que impede ou limita o motor (parada, trava, zero perdido)
 *   ⚡ CORRENTE   limites de corrente e de tensão
 *   🎯 PID        ganhos kP, kI, kD, kS, kV, kG, kA e onde o PID roda
 *   🚀 VELOCIDADE velocidade, aceleração, RPM, perfil e saída em %
 *   📏 MEDIDAS    curso, reduções, geometria e offsets
 *   🧭 POSE       zero, heading, pose e referência do piloto
 *   🎮 BOTÕES     o que cada botão faz
 *   📷 VISÃO      Limelight, filtros e peso das medições
 *   🔁 CICLO      a ordem do que roda a cada 20 ms
 *   🧪 SIMULAÇÃO  física e modelo em paralelo (real x simulado)
 *   📊 TELEMETRIA log, diagnóstico e alertas
 *   🔧 CALIBRAR   rotinas e valores para medir no robô
 *   🚧 ATENÇÃO    ponto do pente fino que merece revisão
 *   📦 ARQUIVADO  código antigo guardado só para consulta
 * Guia completo: ESTRUTURA_DO_CODIGO.md, seção 3c.
 *
 * Só o que é do robô inteiro: as portas dos controles.
 *
 * As constantes de cada subsistema ficam na pasta dele (XxxConstants):
 * drive/DriveConstants, vision/LimelightConstants e VisionConstants,
 * shooter/ShooterConstants, intake/IntakeConstants, e assim por diante.
 * IDs CAN: frc.robot.CanIds. Limites de corrente recomendados: frc.robot.MotorLimits.
 * As anotações das aulas, no fim deste arquivo, foram mantidas.
 */
public final class Constants {
  
  public static class OperatorConstants {
    //Porta onde controle está do xBox
    public static final int kDriverControllerPort = 0;

    // Segundo controle (operador): shooter.
    public static final int kOperatorControllerPort = 1;
  }


}

//=====================================Anotações
//ROBOT CONFIG  = PathPlanner


//Limite tenção Bateria
//As vezes a bateria esta com 13v ou 12 então manda mais doque 12

//slowrate -- Swerve Função quadratica


//Optimize Cocene scale


//================Limelight
//Confiar menos no giroscopio da camera usar o pision
//Pega somente X e Y
//Despreza a rotação
//Usar bloco Try Cath Exception = Limelight 
//Constantes no codigo
//MegaTag 1 não precisa do pision
//Megata 2 precisa do pision sexta posicao e o angulo
//Enquanto for falso que esta vendo a limelight ele fica procurando 
//Quando acha deixa de procurar pega os valores dela
//TimeStamp volta no tempo

//=================Posicao da arena e robo
//Yaw 180 uma alinaça 0 arena uim
//Teste para robo fica lado certo da arena




//=====Sistematiza 
//Monitoramento de Log




//==========Perguntar
//Cabibrar
//Picos Voltagem
//Alinhar quando começa jogar as rodas na posicao 0


//Paphplanner
//Tem como passar o x e y e ele desviar dos objetos automaticamente 
//Ele gera um json com as config do robo peso medidas
//PaphToPose mandar robo por coordenadas 
//PID de trajetoria do robo = robot config


//Java Teste de unidades



//===============================Importanta
//Tests 
//SysId Translator State para para achar PID
