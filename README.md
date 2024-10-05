# sdmap-route-devkit

Launch json for preprocessing:

```
{
    "name": "Create Data Samples",
    "type": "debugpy",
    "request": "launch",
    "program": "${workspaceFolder}/create_data_samples.py",
    "args": [
        "--input=/staging/dl_madmaps/data/dl2_hp_train_default",
        "--output=test/",
        "--workers=1",
        "--debug"
    ],
    "console": "integratedTerminal"
}

```
## Diagram
![Diagram](route_retrieval.drawio.png)
