# Documentación del Proyecto: MANET de Agricultura de Precisión (ns-3.43)

## Descripción general

Este proyecto simula una red MANET jerárquica de dos niveles para agricultura de
precisión utilizando el simulador ns-3.43. El objetivo es estudiar el desempeño
del protocolo **Store-Carry-Forward (SCF)** bajo distintas condiciones de movilidad,
conectividad y configuración del buffer.

---

## Topología de la simulación (`scratch/adhoc.cc`)

### Nodos y jerarquía

| Nivel | Tipo | Cantidad | Rol |
|-------|------|----------|-----|
| 0 | Sensores de campo | fijos (estáticos) | Generan muestras de datos agrícolas |
| 1 | Agrobots | 3 clústeres × 5 nodos | Recolectan datos de sensores vía SCF |
| 2 | UGVs (vehículos terrestres) | 2 clústeres × 3 nodos | Retransmiten datos de agrobots al sink |
| Sink | Casa (house) | 1 nodo fijo | Punto de recolección final de datos |

### Protocolo de comunicación

- **Red**: Wi-Fi ad-hoc de un solo canal (MANET)
- **Enrutamiento**: AODV (Ad hoc On-Demand Distance Vector)
- **Protocolo de datos**: Store-Carry-Forward (SCF) implementado manualmente
  - Cada nodo mantiene un buffer con capacidad configurable (`bufferCapacityPackets`)
  - Los paquetes tienen un tiempo de vida configurable (`sampleTtl`)
  - Cuando dos nodos entran en contacto (dentro del rango `level1ToLevel2ContactRange`),
    los datos se reenvían oportunistamente

### Métricas capturadas

| Métrica | Descripción |
|---------|-------------|
| `house_coverage_pct` | % de sensores del campo cuya muestra llegó a la casa |
| `house_e2e_delay_avg_s` | Delay promedio extremo a extremo (sensor → casa) |
| `house_aoi_avg_s` | Age of Information promedio observado en la casa |
| `stage1_pdr_pct` | PDR etapa 1: sensor → agrobot |
| `stage2_pdr_pct` | PDR etapa 2: agrobot → UGV |
| `stage3_pdr_pct` | PDR etapa 3: UGV → casa |
| `agrobot_avg_buffer_occ` | Ocupación media del buffer SCF en agrobots |
| `ugv_avg_buffer_occ` | Ocupación media del buffer SCF en UGVs |
| `agrobot_ttl_drops` | Paquetes descartados por TTL en agrobots |
| `ugv_ttl_drops` | Paquetes descartados por TTL en UGVs |

---

## Campañas paramétricas

### Campaña 1: Rango de contacto (`contact_range_campaign`)

Estudia cómo varía el desempeño al cambiar la distancia máxima de detección de
vecinos para el reenvío SCF entre agrobots y UGVs.

- **Parámetro**: `--level1ToLevel2ContactRange` (metros)
- **Rango predeterminado**: 5 valores entre 20 m y 150 m
- **Scripts**: `run_contact_range_campaign.py`, `plot_contact_range_campaign.py`

### Campaña 2: Velocidad de nodos (`speed_campaign`)

Estudia el efecto de la movilidad de los nodos seguidores sobre la frecuencia
y duración de los contactos entre nodos.

- **Parámetro**: `--followerSpeedMax` (m/s)
- **Valores predeterminados**: 5.0, 10.0, 15.0 m/s
- **Scripts**: `run_speed_campaign.py`, `plot_speed_campaign.py`

### Campaña 3: TTL y Buffer (`ttl_buffer_campaign`)

Estudio factorial 2D que cruza el tiempo de vida de los paquetes con la capacidad
del buffer SCF.

- **Parámetros**: `--sampleTtl` (s) × `--bufferCapacityPackets` (paquetes)
- **Valores predeterminados**: TTL ∈ {60, 120, 240} s; Buffer ∈ {64, 128, 256}
- **Scripts**: `run_ttl_buffer_campaign.py`, `plot_ttl_buffer_campaign.py`

---

## Estructura de archivos del proyecto

```
ns-3.43/scratch/
├── adhoc.cc                        # Simulación principal (ya documentada)
├── run_contact_range_campaign.py   # Ejecutor de campaña de rango
├── run_speed_campaign.py           # Ejecutor de campaña de velocidad
├── run_ttl_buffer_campaign.py      # Ejecutor de campaña TTL×Buffer
├── campaign_plot_utils.py          # Biblioteca compartida de análisis/visualización
├── plot_contact_range_campaign.py  # Visualizador de campaña de rango
├── plot_speed_campaign.py          # Visualizador de campaña de velocidad
└── plot_ttl_buffer_campaign.py     # Visualizador de campaña TTL×Buffer

ns-3.43/results/                    # Directorio de resultados (no versionado)
├── contact_range_campaign/<ts>/
├── speed_campaign/<ts>/
└── ttl_buffer_campaign/<ts>/
```

---

## Cómo ejecutar

```bash
# Desde la raíz del repositorio ns-3.43

# 1. Compilar la simulación
./ns3 build scratch/adhoc

# 2. Ejecutar una campaña (ejemplo: rango de contacto)
python3 scratch/run_contact_range_campaign.py --runs 10

# 3. Visualizar los resultados
python3 scratch/plot_contact_range_campaign.py
# Abre results/contact_range_campaign/<timestamp>/plots/index.html
```

---

## Instrucciones para agentes de IA

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
