this repo is the driver for the six axis platform, on the application side.

the SDK used to directly drive the platform should be installed seperately.

Config specs

the equalizer uses a yaml config file to equalize the dimensions. the parameters of the equalizer are specified in this section, as follows:

enable: bool, whether this dim is enabled. 

FilterMode: an item that specifies the filtering mode of this dimension. this is a nested item, the parameters of this items are:

-   mode: string, the filter applied. if not specified, by default identity filter is applied.

-   k: int, the filter window. the filter only applies to this window, achieving a pseudo-FIR response. this can be used to adjust granularity and affect filter behavior.

-   filter parameters: this depends on filter. please refer to constants.py for a complete definition of filter required variables.

gain: float, how much gain is applied to this channel. this is applied as the very last step of the equalizer pipeline.