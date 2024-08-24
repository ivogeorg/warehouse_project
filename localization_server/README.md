### `localization_server`

Part of [`warehouse_project`](https://github.com/ivogeorg/warehouse_project.git).

#### Implementation notes

1. Error on bringup:
```
[lifecycle_manager-3] [INFO] [1724457263.335873652] [lifecycle_manager_mapper]: Configuring map_server
[map_server-1] [INFO] [1724457263.336073807] [map_server]:
[map_server-1]  map_server lifecycle node launched.
[map_server-1]  Waiting on external lifecycle transitions to activate
[map_server-1]  See https://design.ros2.org/articles/node_lifecycle.html for more information.
[map_server-1] [INFO] [1724457263.336201255] [map_server]: Creating
[map_server-1] [INFO] [1724457263.336630204] [map_server]: Configuring
[map_server-1] [ERROR] [1724457263.336693035] [map_server]: Caught exception in callback for transition 10
[map_server-1] [ERROR] [1724457263.336708227] [map_server]: Original error: parameter 'yaml_filename' is not initialized
[map_server-1] [WARN] [1724457263.336724888] [map_server]: Error occurred while doing error handling.
[map_server-1] [FATAL] [1724457263.336734366] [map_server]: Lifecycle node map_server does not have error state implemented
[lifecycle_manager-3] [ERROR] [1724457263.337108773] [lifecycle_manager_mapper]: Failed to change state for node: map_server
[lifecycle_manager-3] [ERROR] [1724457263.337134674] [lifecycle_manager_mapper]: Failed to bring up all requested nodes. Aborting bringup.
```