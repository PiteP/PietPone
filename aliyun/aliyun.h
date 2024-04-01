#ifndef _ONENET_H_
#define _ONENET_H_





_Bool AliYun_DevLink(void);

void AliYun_Subscribe(const char *topics[], unsigned char topic_cnt);

void AliYun_Publish(const char *topic, const char *msg);

void AliYun_RevPro(unsigned char *cmd);

#endif
