#include <linux/init.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include "nmi625-snd.h"


#define STD_NMI625_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 |\
		SNDRV_PCM_RATE_48000)

#define SND_SOC_STD_NMI625_FMTS \
	(SNDRV_PCM_FMTBIT_S8 |\
	 SNDRV_PCM_FMTBIT_S16_LE |\
	 SNDRV_PCM_FMTBIT_S16_BE |\
	 SNDRV_PCM_FMTBIT_S20_3LE |\
	 SNDRV_PCM_FMTBIT_S20_3BE |\
	 SNDRV_PCM_FMTBIT_S24_3LE |\
	 SNDRV_PCM_FMTBIT_S24_3BE |\
	 SNDRV_PCM_FMTBIT_S32_LE |\
	 SNDRV_PCM_FMTBIT_S32_BE)

struct snd_soc_dai nmi625_dai = {
	.name = "NMI625 HiFi",
	.capture = {
		.stream_name = "NMI625 Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = STD_NMI625_RATES,
		.formats = SND_SOC_STD_NMI625_FMTS,
	},
};
EXPORT_SYMBOL_GPL(nmi625_dai);


static int nmi625_soc_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_card *card = socdev->card;
	struct snd_soc_codec *codec;
	int ret = 0;

	printk(KERN_INFO "NMI625 SoC Audio Codec\n");

	socdev->card->codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (!socdev->card->codec)
		return -ENOMEM;
	codec = socdev->card->codec;
	mutex_init(&codec->mutex);

	codec->name = "NMI625";
	codec->owner = THIS_MODULE;
	codec->dai = &nmi625_dai;
	codec->num_dai = 1;
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, 2, SNDRV_DEFAULT_STR1);
	if (ret < 0)
		goto err;

	return 0;

bus_err:
	snd_soc_free_pcms(socdev);

err:
	kfree(socdev->card->codec);
	socdev->card->codec = NULL;
	return ret;
}

static int nmi625_soc_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	if (!codec)
		return 0;

	snd_soc_free_pcms(socdev);
	kfree(socdev->card->codec);

	return 0;
}


struct snd_soc_codec_device soc_codec_dev_nmi625 = {
	.probe = 	nmi625_soc_probe,
	.remove = 	nmi625_soc_remove,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_nmi625);


static int __init nmi625_modinit(void)
{
	int ret;
	ret = snd_soc_register_dai(&nmi625_dai);
	if(ret)
		printk(KERN_ERR "nmi625_dai registration failed..\n");

	return ret;
}
module_init(nmi625_modinit);

static void __exit nmi625_exit(void)
{
	snd_soc_unregister_dai(&nmi625_dai);
}
module_exit(nmi625_exit);


MODULE_DESCRIPTION("Soc NMI625 driver");
MODULE_AUTHOR("Ki-Hyung Nam");
MODULE_LICENSE("GPL");

