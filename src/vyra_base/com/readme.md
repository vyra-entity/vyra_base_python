# vyraBase - Communication Component

## Summary:
The communction for vyraBase is resposible to make a V.Y.R.A. module available and reachable for other module in the same domain. As communction framework <b>eProsima fast dds</b> will be used. This implements a publish subscriber structure wich will be used to build <b>V.Y.R.A. callables</b> and <b>observables</b> on top. On one domain will always be just one process scheduler V.Y.R.A. module that is leading the communcation process. 

This process scheduler module is searching for modules in the same domain space that are needed to run the process. Process scheduler of different domains could communicate and share selected module callables or observables. If a process scheduler has found a V.Y.R.A. modules the registration process will be provoked (<b>registration by call</b>). On the other hand V.Y.R.A. modules other than process scheduler could send a registration request to be authorized to the process (<b>registration by request</b>)

---

### Observable:
Observables are attributes of a V.Y.R.A. module that are public and could therefore be watched by other modules

### Callables:
Callables on the other hand are methods of a V.Y.R.A. module that could be executed by a request. At lifetime of the call, it will feedback a <b>process</b> until it finished it. During lifetime it is also possible to <b>manipulate</b> the process by sending additional requests to the callable. This could for example stop the callable or give further information if a callable needs to be redirected.

### Registration


