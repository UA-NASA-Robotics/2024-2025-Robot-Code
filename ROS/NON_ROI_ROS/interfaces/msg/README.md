# Interfaces and Messages
Interfaces are the systems set up that allow programs communicate with each other, in this case how our nodes talk to one another. As mentioned before they use topics, services, and actions to talk, but to define the structure of the data we need messages.

When ever using a topic in a file make sure to import the msg to ensure that it can be decode using the following:
``` Py
from MessageFileName.msg import NameOfMessage
```