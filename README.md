# ES827 — Robótica Industrial: Stäubli TX90

Projeto da disciplina ES827 (FEM/Unicamp): simulação de um Stäubli TX90
pintando a bandeira do Brasil — cinemática, trajetória, dinâmica e controle.

## Estrutura

```
enunciado.pdf          Enunciado do projeto (slides da disciplina)
ata_reuniao_grupo.pdf  Ata da reunião de planejamento do grupo (19/05)
simulacao/             Tudo que é necessário para rodar no MATLAB
  tx90_simulacao.m     Script principal (trajetória, IK, dinâmica, controle, animação)
  tx90.urdf            Modelo do robô (pacote ROS-Industrial staubli_experimental)
  meshes/tx90/         Malhas STL para a visualização 3D
relatorio/             Relatório em LaTeX (projeto do Overleaf)
  main.tex             Documento principal
  Tarefas/             Capítulos e referências (.bib)
  Imagens/             Figuras
```

## Como rodar a simulação

Requisitos: MATLAB com Robotics System Toolbox.

```matlab
cd simulacao
tx90_simulacao
```

O script deve ser executado de dentro de `simulacao/` (ele carrega
`tx90.urdf` por caminho relativo).

## Como compilar o relatório

O `relatorio/` é o projeto do Overleaf (compilar `main.tex` com pdfLaTeX +
BibTeX). Atenção: o listing de código em `Tarefas/resultados.tex` referencia
`../simulacao/tx90_simulacao.m`; ao subir para o Overleaf, copie
`simulacao/tx90_simulacao.m` para dentro do projeto e ajuste esse caminho.
