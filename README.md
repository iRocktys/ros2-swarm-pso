# Beswarm1.0

O Beswarm-1.0 é um repositório que apresenta a utilização do algoritmo de otimização bioinspirado **Particle Swarm Optimization (PSO)** para realizar a movimentação de múltiplos robôs na plataforma **ROS 2 Iron**. O principal objetivo do projeto é avaliar a eficácia deste algoritmo para a movimentação autônoma de robôs em ambiente de simulação ou físico.

## Objetivo

Implementar e avaliar o desempenho do algoritmo PSO para coordenar a movimentação de robôs móveis. A solução foi desenvolvida utilizando a plataforma ROS 2 Iron e testada em ambiente de simulação, utilizando ferramentas como o TurtleSim e o rqt.

## Estrutura do Repositório

- **Código**: Contém os scripts de código utilizados para implementar o PSO nos robôs e simulações com ROS 2 Iron.
- **Tutoriais**: Nesta pasta, estão os arquivos que orientam a configuração do ambiente de simulação e o uso da plataforma. Eles incluem:
  - **Comandos básicos, TurtleSim e rqt.pdf**: Explicação dos comandos básicos e configuração do simulador TurtleSim e da interface rqt.
  - **Instalação ROS2 Iron.pdf**: Tutorial detalhado para a instalação do ROS 2 Iron.
  - **VirtualBox.pdf**: Guia para configurar o VirtualBox para uso com o ROS 2 e simulação de robôs.

## Requisitos

Para rodar este projeto, é necessário:

- **ROS 2 Iron** instalado (vide tutorial na pasta Tutoriais).
- Ambiente de simulação configurado com o **TurtleSim** ou outro simulador compatível com ROS 2.
- **Python 3.8+** e pacotes necessários listados no código.

## Instalação

Siga os passos abaixo para configurar o ambiente:

1. Instale o **ROS 2 Iron** seguindo as instruções fornecidas no arquivo [Instalação ROS2 Iron.pdf](./Tutoriais/Instalação%20ROS2%20Iron.pdf).
2. Configure o simulador **TurtleSim** e o **rqt** conforme o tutorial [Comandos básicos, TurtleSim e rqt.pdf](./Tutoriais/Comandos%20básicos,%20TurtleSim%20e%20rqt.pdf).
3. (Opcional) Se necessário, configure um ambiente virtual com o **VirtualBox**, conforme o guia [VirtualBox.pdf](./Tutoriais/VirtualBox.pdf).

