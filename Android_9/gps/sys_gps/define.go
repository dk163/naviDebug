package define

import (
        "android/soong/android"
        "android/soong/cc"
        "fmt"
)

func init() {
    // for DEBUG
    fmt.Println("sys init start")
    android.RegisterModuleType("sys_gps_defaults", sys_gpsDefaultsFactory)
}

func sys_gpsDefaultsFactory() (android.Module) {
    module := cc.DefaultsFactory()
    android.AddLoadHook(module, sys_gpsDefaults)
    return module
}

func sys_gpsDefaults(ctx android.LoadHookContext) {
    type props struct {
        Cflags []string
    }
    p := &props{}
    p.Cflags = globalDefaults(ctx)
    ctx.AppendProperties(p)
}

func globalDefaults(ctx android.BaseContext) ([]string) {
    var cppflags []string
    
    fmt.Println("DeviceName:", ctx.AConfig().DeviceName())

    name := ctx.AConfig().DeviceName();
        
    if name == "chinatsp_s203_p_8q" {
          cppflags = append(cppflags,
                         "-DCHINATSP_S203_P_8Q=\"chinatsp_s203_p_8q\"")
    }
    
    if name == "chinatsp_f202_p_8q" {
          cppflags = append(cppflags,
                         "-DCHINATSP_F202_P_8Q=\"chinatsp_f202_p_8q\"")
    }

    return cppflags
}
