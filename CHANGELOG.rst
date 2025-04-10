^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package remap_plugin_query
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* basic README
* allowing publishing empty point clouds
  unfortunately, I can't remember why I introduced in the first
  place this check. I remember something was crashing. However,
  things are working now, and it makes sense to publish an
  empty point cloud in the case the result of the query is... empty.
  You might see me reverting this commit in the future.
* making linters happy
* default queries
* supporting tf publishing for static queries
* more features (check full commit message)
  - it is now possible to add non-stopping queries by setting a
  negative duration time
  - it is now possible to remove queries; this is supported via
  a new custom service message (remap_msgs/srv/RemoveQuery)
* publishing tfs
* enabled dynamic querying
* Initial commit
* Contributors: Lorenzo Ferrini, lorenzoferrini
