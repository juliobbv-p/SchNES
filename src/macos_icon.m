#import <AppKit/AppKit.h>

void schnes_set_dock_icon(void)
{
    @autoreleasepool {
        NSBundle *bundle = NSBundle.mainBundle;
        NSString *name = [bundle objectForInfoDictionaryKey:@"CFBundleIconFile"];

        if (!name)
            return;
        NSURL *url = [bundle URLForResource:name.stringByDeletingPathExtension
                             withExtension:name.pathExtension];
        NSImage *icon = url ? [[NSImage alloc] initWithContentsOfURL:url] : nil;

        if (icon.valid)
            NSApplication.sharedApplication.applicationIconImage = icon;
    }
}
