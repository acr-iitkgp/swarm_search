Documentaion:

these are the raw source files of the build. THESE ARE NOT REQUIRED FOR FUNCTIONALITY!

NOTE: Npm cannot work with ros. Because they have common dependencies at libssl1.0-dev See- https://github.com/apollographql/graphql-subscriptions/issues/83#issuecomment-326675684

Since there is no workaround, to build, force install npm which would delete 18 Mbs worth of ros libraries and then build test your install, and then reinstall ros to again run server (Hectic, YES.)
