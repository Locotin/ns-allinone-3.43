Usa siempre Context7 cuando necesites documentacion de librerias, APIs, pasos de configuracion o generacion de codigo, sin esperar a que el usuario lo pida explicitamente.

Cuando trabajes con ns-3 o librerias externas, prioriza Context7 para obtener documentacion y ejemplos actualizados antes de responder o generar cambios.

Valida siempre ejemplos, simbolos y APIs contra la version local del proyecto, que en este taller es ns-3.43. Si Context7 muestra referencias de otra version, adapta la solucion antes de proponer o implementar codigo.

Prioriza cambios en `scratch/` para experimentos y entregables del taller. No modifiques `src/`, modulos base o infraestructura del simulador salvo que el usuario lo pida de forma explicita.

Preserva la reproducibilidad de las simulaciones. No cambies silenciosamente parametros experimentales como `rngRun`, tiempos, tasas, tamanos de paquete, cantidad de nodos o nombres de archivos de salida.

Cuando modifiques una simulacion, compila y ejecuta al menos una corrida corta de verificacion antes de cerrar el trabajo para validar build, parseo de linea de comandos y arranque basico de la simulacion.

Si agregas metricas, trazas o salidas, indica claramente si provienen de `FlowMonitor`, logs, trazas de ns-3 o calculos manuales, y deja consistente la forma de reportarlas.

No borres ni sobrescribas artefactos existentes como `.xml`, `.tr` o `.pcap` sin instruccion explicita. Prefiere respetar parametros CLI existentes o usar nuevos nombres de salida.

Usa comentarios orientados al experimento: topologia, hipotesis, trafico, parametros y criterios de medicion. Evita comentarios triviales que solo describan sintaxis de C++.

Si un cambio altera el significado metodologico del experimento o puede modificar resultados, explicitalo como cambio experimental, no solo como cambio de implementacion.
