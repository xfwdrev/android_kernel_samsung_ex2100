/*
 * s2dos05.c - Regulator driver for the Samsung s2dos05
 *
 * Copyright (C) 2016 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/module.h>
#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/s2dos05.h>
#include <linux/regulator/of_regulator.h>
#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
#include <linux/regulator/pmic_class.h>
#endif
#if IS_ENABLED(CONFIG_REGULATOR_DEBUG_CONTROL)
#include <linux/regulator/debug-regulator.h>
#endif
#if IS_ENABLED(CONFIG_SEC_PM)
#include <linux/sec_class.h>
#include <linux/fb.h>
#endif /* CONFIG_SEC_PM */

#if IS_ENABLED(CONFIG_SEC_ABC)
#include <linux/sti/abc_common.h>
#endif /* CONFIG_SEC_ABC */

#if IS_ENABLED(CONFIG_SEC_PM_QCOM)
extern int msm_drm_register_notifier_client(struct notifier_block *nb);
extern int msm_drm_unregister_notifier_client(struct notifier_block *nb);
#endif

struct s2dos05_data {
	struct s2dos05_dev *iodev;
	int num_regulators;
	struct regulator_dev *rdev[S2DOS05_REGULATOR_MAX];
#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
	u8 read_addr;
	u8 read_val;
	struct device *dev;
#endif
#if IS_ENABLED(CONFIG_SEC_PM)
	struct notifier_block fb_block __maybe_unused;
	struct delayed_work fd_work __maybe_unused;
	bool fd_work_init;
#endif /* CONFIG_SEC_PM */
#if IS_ENABLED(CONFIG_SEC_ABC)
	atomic_t i2c_fail_count;
#endif /* CONFIG_SEC_ABC */
};

#if IS_ENABLED(CONFIG_SEC_ABC)
#define s2dos05_i2c_abc_event(arg...)	\
	__s2dos05_i2c_abc_event((char *)__func__, ##arg)

static void __s2dos05_i2c_abc_event(char *func, struct s2dos05_data *info, int ret)
{
	char buf[64];
	char *type;
	int count;
	const int fail_threshold_cnt = 2;

#if IS_ENABLED(CONFIG_SEC_FACTORY)
	type = "INFO";
#else
	type = "WARN";
#endif /* CONFIG_SEC_FACTORY */

	if (ret < 0) {
		count = atomic_inc_return(&info->i2c_fail_count);
		if (count >= fail_threshold_cnt) {
			atomic_set(&info->i2c_fail_count, 0);
			snprintf(buf, sizeof(buf), "MODULE=pmic@%s=%s_fail", type, func);
			sec_abc_send_event(buf);
		}
	} else {
		atomic_set(&info->i2c_fail_count, 0);
	}
}

static void s2dos05_irq_abc_event(const char *irq_desc)
{
	char buf[64];
	char *type;

#if IS_ENABLED(CONFIG_SEC_FACTORY)
	type = "WARN";
#else
	type = "WARN";	/* Diamond: black screen issue in user binary */
#endif /* CONFIG_SEC_FACTORY */

	snprintf(buf, sizeof(buf), "MODULE=pmic@%s=s2dos05_%s", type, irq_desc);
	sec_abc_send_event(buf);
}
#else
#define s2dos05_i2c_abc_event(arg...)	do {} while (0)
#define s2dos05_irq_abc_event(arg...)	do {} while (0)
#endif /* CONFIG_SEC_ABC */

int s2dos05_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct s2dos05_data *info = i2c_get_clientdata(i2c);
	struct s2dos05_dev *s2dos05 = info->iodev;
	int ret;

	mutex_lock(&s2dos05->i2c_lock);
	ret = i2c_smbus_read_byte_data(i2c, reg);
	mutex_unlock(&s2dos05->i2c_lock);
	s2dos05_i2c_abc_event(info, ret);
	if (ret < 0) {
		pr_info("%s:%s reg(0x%02hhx), ret(%d)\n",
			 MFD_DEV_NAME, __func__, reg, ret);
		return ret;
	}

	ret &= 0xff;
	*dest = ret;
	return 0;
}
EXPORT_SYMBOL_GPL(s2dos05_read_reg);

int s2dos05_bulk_read(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct s2dos05_data *info = i2c_get_clientdata(i2c);
	struct s2dos05_dev *s2dos05 = info->iodev;
	int ret;

	mutex_lock(&s2dos05->i2c_lock);
	ret = i2c_smbus_read_i2c_block_data(i2c, reg, count, buf);
	mutex_unlock(&s2dos05->i2c_lock);
	s2dos05_i2c_abc_event(info, ret);
	if (ret < 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(s2dos05_bulk_read);

int s2dos05_read_word(struct i2c_client *i2c, u8 reg)
{
	struct s2dos05_data *info = i2c_get_clientdata(i2c);
	struct s2dos05_dev *s2dos05 = info->iodev;
	int ret;

	mutex_lock(&s2dos05->i2c_lock);
	ret = i2c_smbus_read_word_data(i2c, reg);
	mutex_unlock(&s2dos05->i2c_lock);
	s2dos05_i2c_abc_event(info, ret);
	if (ret < 0)
		return ret;

	return ret;
}
EXPORT_SYMBOL_GPL(s2dos05_read_word);

int s2dos05_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct s2dos05_data *info = i2c_get_clientdata(i2c);
	struct s2dos05_dev *s2dos05 = info->iodev;
	int ret;

	mutex_lock(&s2dos05->i2c_lock);
	ret = i2c_smbus_write_byte_data(i2c, reg, value);
	mutex_unlock(&s2dos05->i2c_lock);
	s2dos05_i2c_abc_event(info, ret);
	if (ret < 0)
		pr_info("%s:%s reg(0x%02hhx), ret(%d)\n",
				MFD_DEV_NAME, __func__, reg, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(s2dos05_write_reg);

int s2dos05_bulk_write(struct i2c_client *i2c, u8 reg, int count, u8 *buf)
{
	struct s2dos05_data *info = i2c_get_clientdata(i2c);
	struct s2dos05_dev *s2dos05 = info->iodev;
	int ret;

	mutex_lock(&s2dos05->i2c_lock);
	ret = i2c_smbus_write_i2c_block_data(i2c, reg, count, buf);
	mutex_unlock(&s2dos05->i2c_lock);
	s2dos05_i2c_abc_event(info, ret);
	if (ret < 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(s2dos05_bulk_write);

int s2dos05_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
	struct s2dos05_data *info = i2c_get_clientdata(i2c);
	struct s2dos05_dev *s2dos05 = info->iodev;
	int ret;
	u8 old_val, new_val;

	mutex_lock(&s2dos05->i2c_lock);
	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret >= 0) {
		old_val = ret & 0xff;
		new_val = (val & mask) | (old_val & (~mask));
		ret = i2c_smbus_write_byte_data(i2c, reg, new_val);
	}
	mutex_unlock(&s2dos05->i2c_lock);
	s2dos05_i2c_abc_event(info, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(s2dos05_update_reg);

static int s2m_enable(struct regulator_dev *rdev)
{
	struct s2dos05_data *info = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = info->iodev->i2c;

	return s2dos05_update_reg(i2c, rdev->desc->enable_reg,
					rdev->desc->enable_mask,
					rdev->desc->enable_mask);
}

static int s2m_disable_regmap(struct regulator_dev *rdev)
{
	struct s2dos05_data *info = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = info->iodev->i2c;
	u8 val;

	if (rdev->desc->enable_is_inverted)
		val = rdev->desc->enable_mask;
	else
		val = 0;

	return s2dos05_update_reg(i2c, rdev->desc->enable_reg,
				   val, rdev->desc->enable_mask);
}

static int s2m_is_enabled_regmap(struct regulator_dev *rdev)
{
	struct s2dos05_data *info = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = info->iodev->i2c;
	int ret;
	u8 val;

	ret = s2dos05_read_reg(i2c, rdev->desc->enable_reg, &val);
	if (ret < 0)
		return ret;

	if (rdev->desc->enable_is_inverted)
		return (val & rdev->desc->enable_mask) == 0;
	else
		return (val & rdev->desc->enable_mask) != 0;
}

static int s2m_get_voltage_sel_regmap(struct regulator_dev *rdev)
{
	struct s2dos05_data *info = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = info->iodev->i2c;
	int ret;
	u8 val;

	ret = s2dos05_read_reg(i2c, rdev->desc->vsel_reg, &val);
	if (ret < 0)
		return ret;

	val &= rdev->desc->vsel_mask;

	return val;
}

static int s2m_set_voltage_sel_regmap(struct regulator_dev *rdev, unsigned sel)
{
	struct s2dos05_data *info = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = info->iodev->i2c;
	int ret;

	ret = s2dos05_update_reg(i2c, rdev->desc->vsel_reg,
				sel, rdev->desc->vsel_mask);
	if (ret < 0)
		goto out;

	if (rdev->desc->apply_bit)
		ret = s2dos05_update_reg(i2c, rdev->desc->apply_reg,
					 rdev->desc->apply_bit,
					 rdev->desc->apply_bit);
	return ret;
out:
	pr_warn("%s: failed to set voltage_sel_regmap\n", rdev->desc->name);
	return ret;
}

static int s2m_set_voltage_sel_regmap_buck(struct regulator_dev *rdev,
					unsigned sel)
{
	struct s2dos05_data *info = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = info->iodev->i2c;
	int ret;

	ret = s2dos05_write_reg(i2c, rdev->desc->vsel_reg, sel);
	if (ret < 0)
		goto out;

	if (rdev->desc->apply_bit)
		ret = s2dos05_update_reg(i2c, rdev->desc->apply_reg,
					 rdev->desc->apply_bit,
					 rdev->desc->apply_bit);
	return ret;
out:
	pr_warn("%s: failed to set voltage_sel_regmap\n", rdev->desc->name);
	return ret;
}

static int s2m_set_voltage_time_sel(struct regulator_dev *rdev,
				   unsigned int old_selector,
				   unsigned int new_selector)
{
	int old_volt, new_volt;

	/* sanity check */
	if (!rdev->desc->ops->list_voltage)
		return -EINVAL;

	old_volt = rdev->desc->ops->list_voltage(rdev, old_selector);
	new_volt = rdev->desc->ops->list_voltage(rdev, new_selector);

	if (old_selector < new_selector)
		return DIV_ROUND_UP(new_volt - old_volt, S2DOS05_RAMP_DELAY);

	return 0;
}

#if IS_ENABLED(CONFIG_SEC_PM)
static int s2m_ssd_convert_uA_to_reg_val(bool enable, int min_uA, int max_uA,
					u8 *val, u8 *mask)
{
	u8 sel = 0;

	if (enable) {
		while (2000 * (sel + 1) < min_uA)
			sel++;

		if (2000 * (sel + 1) > max_uA)
			return -EINVAL;
	}

	*val = (!enable << 3) | (sel << 5);
	*mask = S2DOS05_ELVSS_SSD_EN_MASK | S2DOS05_ELVSS_SEL_SSD_MASK;

	return 0;
}


static int s2m_set_elvss_ssd_current_limit(struct regulator_dev *rdev,
					int min_uA, int max_uA)
{
	struct s2dos05_data *info = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = info->iodev->i2c;
	int ret;
	bool enable;
	u8 val = 0, mask = 0;

	enable = min_uA || max_uA;

	ret = s2m_ssd_convert_uA_to_reg_val(enable, min_uA, max_uA, &val, &mask);
	if (ret < 0)
		return ret;

	ret = s2dos05_update_reg(i2c, S2DOS05_REG_SSD_TSD, val, mask);
	return ret;
}

static int s2m_get_elvss_ssd_current_limit(struct regulator_dev *rdev)
{
	struct s2dos05_data *info = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = info->iodev->i2c;
	int ret;
	bool enable;
	u8 val;

	ret = s2dos05_read_reg(i2c, S2DOS05_REG_SSD_TSD, &val);
	if (ret < 0)
		return ret;

	enable = !(val & S2DOS05_ELVSS_SSD_EN_MASK);

	if (!enable)
		return 0;

	ret = (val & S2DOS05_ELVSS_SEL_SSD_MASK) >> 5;
	return (ret + 1) * 2000;
}

#if IS_ENABLED(CONFIG_ARCH_QCOM)
static int s2m_elvss_fd_is_enabled(struct regulator_dev *rdev)
{
	/* Always return false due to timing issue */
	return 0;
}
#else
static int s2m_elvss_fd_is_enabled(struct regulator_dev *rdev)
{
	struct s2dos05_data *info = rdev_get_drvdata(rdev);
	struct i2c_client *i2c = info->iodev->i2c;
	int ret;
	u8 val;

	ret = s2dos05_read_reg(i2c, S2DOS05_REG_UVLO_FD, &val);
	if (ret < 0) {
		/* If failed to read FD status, suppose it is not enabled */
		pr_info("%s: fail to read i2c address\n", __func__);
		return 0;
	}

	return !(val & 0x1);
}
#endif

#define DEFAULT_ENABLE_FD_DELAY_MS	500

static int s2m_elvss_fd_enable(struct regulator_dev *rdev)
{
	int ret = 0;
	struct s2dos05_data *info = rdev_get_drvdata(rdev);
	unsigned int delay = info->iodev->pdata->enable_fd_delay_ms;

	if (delay)
		delay = msecs_to_jiffies(delay);

	/* To guarantee fd_work is initialized */
	if (info->fd_work_init) {
		ret = schedule_delayed_work(&info->fd_work, delay);
		if(!ret)
			pr_info("%s: schedule_delayed_work error!\n", __func__);
	}

	return ret;
}

static int s2m_elvss_fd_disable(struct regulator_dev *rdev)
{
	struct s2dos05_data *info = rdev_get_drvdata(rdev);

	if (info->fd_work_init)
		cancel_delayed_work_sync(&info->fd_work);

	return 0;
}
#endif /* CONFIG_SEC_PM */

static struct regulator_ops s2dos05_ldo_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= s2m_is_enabled_regmap,
	.enable			= s2m_enable,
	.disable		= s2m_disable_regmap,
	.get_voltage_sel	= s2m_get_voltage_sel_regmap,
	.set_voltage_sel	= s2m_set_voltage_sel_regmap,
	.set_voltage_time_sel	= s2m_set_voltage_time_sel,
};

static struct regulator_ops s2dos05_buck_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= s2m_is_enabled_regmap,
	.enable			= s2m_enable,
	.disable		= s2m_disable_regmap,
	.get_voltage_sel	= s2m_get_voltage_sel_regmap,
	.set_voltage_sel	= s2m_set_voltage_sel_regmap_buck,
	.set_voltage_time_sel	= s2m_set_voltage_time_sel,
};

#if IS_ENABLED(CONFIG_SEC_PM)
static struct regulator_ops s2dos05_elvss_ssd_ops = {
	.set_current_limit	= s2m_set_elvss_ssd_current_limit,
	.get_current_limit	= s2m_get_elvss_ssd_current_limit,
};

static struct regulator_ops s2dos05_elvss_fd_ops = {
	.is_enabled		= s2m_elvss_fd_is_enabled,
	.enable			= s2m_elvss_fd_enable,
	.disable		= s2m_elvss_fd_disable,
};
#endif /* CONFIG_SEC_PM */

#define _BUCK(macro)	S2DOS05_BUCK##macro
#define _buck_ops(num)	s2dos05_buck_ops##num

#define _LDO(macro)	S2DOS05_LDO##macro
#define _REG(ctrl)	S2DOS05_REG##ctrl
#define _ldo_ops(num)	s2dos05_ldo_ops##num
#define _MASK(macro)	S2DOS05_ENABLE_MASK##macro
#define _TIME(macro)	S2DOS05_ENABLE_TIME##macro

#define BUCK_DESC(_name, _id, _ops, m, s, v, e, em, t)	{	\
	.name		= _name,				\
	.id		= _id,					\
	.ops		= _ops,					\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= m,					\
	.uV_step	= s,					\
	.n_voltages	= S2DOS05_BUCK_N_VOLTAGES,		\
	.vsel_reg	= v,					\
	.vsel_mask	= S2DOS05_BUCK_VSEL_MASK,		\
	.enable_reg	= e,					\
	.enable_mask	= em,					\
	.enable_time	= t					\
}

#define LDO_DESC(_name, _id, _ops, m, s, v, e, em, t)	{	\
	.name		= _name,				\
	.id		= _id,					\
	.ops		= _ops,					\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= m,					\
	.uV_step	= s,					\
	.n_voltages	= S2DOS05_LDO_N_VOLTAGES,		\
	.vsel_reg	= v,					\
	.vsel_mask	= S2DOS05_LDO_VSEL_MASK,		\
	.enable_reg	= e,					\
	.enable_mask	= em,					\
	.enable_time	= t					\
}

#if IS_ENABLED(CONFIG_SEC_PM)
#define ELVSS_DESC(_name, _id)				{	\
	.name		= _name,				\
	.id		= _id,					\
	.ops		= &s2dos05_elvss_ssd_ops,			\
	.type		= REGULATOR_CURRENT,			\
	.owner		= THIS_MODULE,				\
}

#define ELVSS_FD_DESC(_name, _id)				{	\
	.name		= _name,				\
	.id		= _id,					\
	.ops		= &s2dos05_elvss_fd_ops,			\
	.type		= REGULATOR_CURRENT,			\
	.owner		= THIS_MODULE,				\
}
#endif /* CONFIG_SEC_PM */

static struct regulator_desc regulators[S2DOS05_REGULATOR_MAX] = {
	/* name, id, ops, min_uv, uV_step, vsel_reg, enable_reg */
	LDO_DESC("s2dos05-ldo1", _LDO(1), &_ldo_ops(), _LDO(_MIN1),
		_LDO(_STEP1), _REG(_LDO1_CFG),
		_REG(_EN), _MASK(_L1), _TIME(_LDO)),
	LDO_DESC("s2dos05-ldo2", _LDO(2), &_ldo_ops(), _LDO(_MIN1),
		_LDO(_STEP1), _REG(_LDO2_CFG),
		_REG(_EN), _MASK(_L2), _TIME(_LDO)),
	LDO_DESC("s2dos05-ldo3", _LDO(3), &_ldo_ops(), _LDO(_MIN2),
		_LDO(_STEP1), _REG(_LDO3_CFG),
		_REG(_EN), _MASK(_L3), _TIME(_LDO)),
	LDO_DESC("s2dos05-ldo4", _LDO(4), &_ldo_ops(), _LDO(_MIN2),
		_LDO(_STEP1), _REG(_LDO4_CFG),
		_REG(_EN), _MASK(_L4), _TIME(_LDO)),
	BUCK_DESC("s2dos05-buck1", _BUCK(1), &_buck_ops(), _BUCK(_MIN1),
		_BUCK(_STEP1), _REG(_BUCK_VOUT),
		_REG(_EN), _MASK(_B1), _TIME(_BUCK)),
#if IS_ENABLED(CONFIG_SEC_PM)
	ELVSS_DESC("s2dos05-elvss-ssd", S2DOS05_ELVSS_SSD),
	ELVSS_FD_DESC("s2dos05-avdd-elvdd-elvss-fd", S2DOS05_ELVSS_FD),
#endif /* CONFIG_SEC_PM */
};

#if IS_ENABLED(CONFIG_SEC_PM)
static void s2dos05_irq_notifier_call_chain(struct s2dos05_data *s2dos05,
					u8 reg_val)
{
	int i;
	int data = reg_val;

	for (i = 0; i < s2dos05->num_regulators; i++)
		regulator_notifier_call_chain(s2dos05->rdev[i],
				REGULATOR_EVENT_FAIL, (void *)&data);
}
#endif /* CONFIG_SEC_PM */

static irqreturn_t s2dos05_irq_thread(int irq, void *irq_data)
{
	struct s2dos05_data *s2dos05 = irq_data;
	u8 val = 0;
#if IS_ENABLED(CONFIG_SEC_PM)
	u8 scp_val[2] = { 0, };
	const char *irq_bit[] = { "ocd", "uvlo", "scp", "ssd", "tsd", "pwrmt" };
	char irq_name[32];
	ssize_t ret = 0;
	unsigned long bit, tmp;
#endif /* CONFIG_SEC_PM */

	s2dos05_read_reg(s2dos05->iodev->i2c, S2DOS05_REG_IRQ, &val);
	pr_info("%s:irq(%d) S2DOS05_REG_IRQ : 0x%02hhx\n", __func__, irq, val);

#if IS_ENABLED(CONFIG_SEC_PM)
	tmp = val;
	for_each_set_bit(bit, &tmp, ARRAY_SIZE(irq_bit)) {
		ret += sprintf(irq_name + ret, " %s", irq_bit[bit]);
		s2dos05_irq_abc_event(irq_bit[bit]);
	}

	pr_info("%s: irq:%s\n", __func__, irq_name);

	/* Show which regulator's SCP occurs */
	if (0x04 & val) {
		if (s2dos05->iodev->is_sm3080) {
			/* SM3080 */
			s2dos05_read_reg(s2dos05->iodev->i2c, INT_STATUS1, &scp_val[0]);
			pr_info("%s:INT_STATUS1(0x%02hhx)\n", __func__, scp_val[0]);
		} else {
			/* S2DOS05 */
			s2dos05_read_reg(s2dos05->iodev->i2c, FAULT_STATUS1, &scp_val[0]);
			s2dos05_read_reg(s2dos05->iodev->i2c, FAULT_STATUS2, &scp_val[1]);
			pr_info("%s:FAULT_STATUS1(0x%02hhx), FAULT_STATUS2(0x%02hhx)\n", __func__, scp_val[0], scp_val[1]);
		}
	}
#endif /* CONFIG_SEC_PM */

#if IS_ENABLED(CONFIG_SEC_PM)
	s2dos05_irq_notifier_call_chain(s2dos05, val);
#endif /* CONFIG_SEC_PM */

	return IRQ_HANDLED;
}

#if IS_ENABLED(CONFIG_OF)
static int s2dos05_pmic_dt_parse_pdata(struct device *dev,
					struct s2dos05_platform_data *pdata)
{
	struct device_node *pmic_np, *regulators_np, *reg_np;
	struct s2dos05_regulator_data *rdata;
	size_t i;
	int ret;
	u32 val;

	pmic_np = dev->of_node;
	if (!pmic_np) {
		dev_err(dev, "could not find pmic sub-node\n");
		return -ENODEV;
	}

	pdata->dp_pmic_irq = of_get_named_gpio(pmic_np, "s2dos05,s2dos05_int", 0);
	if (pdata->dp_pmic_irq < 0)
		pr_err("%s error reading s2dos05_irq = %d\n",
			__func__, pdata->dp_pmic_irq);

	pdata->wakeup = of_property_read_bool(pmic_np, "s2dos05,wakeup");

#if IS_ENABLED(CONFIG_SEC_PM)
	if (!of_property_read_string(pmic_np, "sec_disp_pmic_name",
				&pdata->sec_disp_pmic_name))
		dev_info(dev, "sec_disp_pmic_name: %s\n",
				pdata->sec_disp_pmic_name);
#endif /* CONFIG_SEC_PM */

	pdata->adc_mode = 0;
	ret = of_property_read_u32(pmic_np, "adc_mode", &val);
	if (ret)
		return -EINVAL;
	pdata->adc_mode = val;

	pdata->adc_sync_mode = 0;
	ret = of_property_read_u32(pmic_np, "adc_sync_mode", &val);
	if (ret)
		return -EINVAL;
	pdata->adc_sync_mode = val;

#if IS_ENABLED(CONFIG_SEC_PM)
	pdata->ocl_elvss = -1;
	ret = of_property_read_u32(pmic_np, "ocl_elvss", &val);
	if (!ret) {
		pdata->ocl_elvss = val;
		dev_info(dev, "get ocl elvss value: %d\n", pdata->ocl_elvss);
	}

	pdata->enable_fd_delay_ms = DEFAULT_ENABLE_FD_DELAY_MS;
	ret = of_property_read_u32(pmic_np, "enable_fd_delay_ms", &val);
	if (!ret) {
		pdata->enable_fd_delay_ms = val;
		dev_info(dev, "enable_fd_delay_ms: %u\n", pdata->enable_fd_delay_ms);
	}
#endif

	regulators_np = of_find_node_by_name(pmic_np, "regulators");
	if (!regulators_np) {
		dev_err(dev, "could not find regulators sub-node\n");
		return -EINVAL;
	}

	/* count the number of regulators to be supported in pmic */
	pdata->num_regulators = 0;
	for_each_child_of_node(regulators_np, reg_np) {
		pdata->num_regulators++;
	}

	rdata = devm_kzalloc(dev, sizeof(*rdata) *
				pdata->num_regulators, GFP_KERNEL);
	if (!rdata) {
		dev_err(dev,
			"could not allocate memory for regulator data\n");
		return -ENOMEM;
	}

	pdata->regulators = rdata;
	pdata->num_rdata = 0;
	for_each_child_of_node(regulators_np, reg_np) {
		for (i = 0; i < ARRAY_SIZE(regulators); i++)
			if (!of_node_cmp(reg_np->name,
					regulators[i].name))
				break;

		if (i == ARRAY_SIZE(regulators)) {
			dev_warn(dev,
			"don't know how to configure regulator %s\n",
			reg_np->name);
			continue;
		}

		rdata->id = i;
		rdata->initdata = of_get_regulator_init_data(
						dev, reg_np,
						&regulators[i]);
		rdata->reg_node = reg_np;
		rdata++;
		pdata->num_rdata++;
	}
	of_node_put(regulators_np);

	return 0;
}
#else
static int s2dos05_pmic_dt_parse_pdata(struct s2dos05_dev *iodev,
					struct s2dos05_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */
#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
static ssize_t s2dos05_read_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct s2dos05_data *s2dos05 = dev_get_drvdata(dev);
	int ret;
	u8 val, reg_addr;

	if (buf == NULL) {
		pr_info("%s: empty buffer\n", __func__);
		return -1;
	}

	ret = kstrtou8(buf, 0, &reg_addr);
	if (ret < 0)
		pr_info("%s: fail to transform i2c address\n", __func__);

	ret = s2dos05_read_reg(s2dos05->iodev->i2c, reg_addr, &val);
	if (ret < 0)
		pr_info("%s: fail to read i2c address\n", __func__);

	pr_info("%s: reg(0x%02hhx) data(0x%02hhx)\n", __func__, reg_addr, val);
	s2dos05->read_addr = reg_addr;
	s2dos05->read_val = val;

	return size;
}

static ssize_t s2dos05_read_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct s2dos05_data *s2dos05 = dev_get_drvdata(dev);
	return sprintf(buf, "0x%02hhx: 0x%02hhx\n", s2dos05->read_addr,
		       s2dos05->read_val);
}

static ssize_t s2dos05_write_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct s2dos05_data *s2dos05 = dev_get_drvdata(dev);
	int ret;
	u8 reg = 0, data = 0;

	if (buf == NULL) {
		pr_info("%s: empty buffer\n", __func__);
		return size;
	}

	ret = sscanf(buf, "0x%02hhx 0x%02hhx", &reg, &data);
	if (ret != 2) {
		pr_info("%s: input error\n", __func__);
		return size;
	}

	pr_info("%s: reg(0x%02hhx) data(0x%02hhx)\n", __func__, reg, data);

	ret = s2dos05_write_reg(s2dos05->iodev->i2c, reg, data);
	if (ret < 0)
		pr_info("%s: fail to write i2c addr/data\n", __func__);

	return size;
}

static ssize_t s2dos05_write_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	return sprintf(buf, "echo (register addr.) (data) > s2dos05_write\n");
}

#define ATTR_REGULATOR	(2)
static struct pmic_device_attribute regulator_attr[] = {
	PMIC_ATTR(s2dos05_write, S_IRUGO | S_IWUSR, s2dos05_write_show, s2dos05_write_store),
	PMIC_ATTR(s2dos05_read, S_IRUGO | S_IWUSR, s2dos05_read_show, s2dos05_read_store),
};

static int s2dos05_create_sysfs(struct s2dos05_data *s2dos05)
{
	struct device *s2dos05_pmic = s2dos05->dev;
	struct device *dev = s2dos05->iodev->dev;
	char device_name[32] = {0, };
	int err = -ENODEV, i = 0;

	pr_info("%s()\n", __func__);
	s2dos05->read_addr = 0;
	s2dos05->read_val = 0;

	/* Dynamic allocation for device name */
	snprintf(device_name, sizeof(device_name) - 1, "%s@%s",
		 dev_driver_string(dev), dev_name(dev));

	s2dos05_pmic = pmic_device_create(s2dos05, device_name);
	s2dos05->dev = s2dos05_pmic;

	/* Create sysfs entries */
	for (i = 0; i < ATTR_REGULATOR; i++) {
		err = device_create_file(s2dos05_pmic, &regulator_attr[i].dev_attr);
		if (err)
			goto remove_pmic_device;
	}

	return 0;

remove_pmic_device:
	for (i--; i >= 0; i--)
		device_remove_file(s2dos05_pmic, &regulator_attr[i].dev_attr);
	pmic_device_destroy(s2dos05_pmic->devt);

	return -1;
}
#endif

#if IS_ENABLED(CONFIG_SEC_PM)
static ssize_t enable_fd_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct s2dos05_data *info = dev_get_drvdata(dev);
	struct i2c_client *i2c = info->iodev->i2c;
	u8 uvlo_fd;
	bool enabled;

	s2dos05_read_reg(i2c, S2DOS05_REG_UVLO_FD, &uvlo_fd);

	dev_info(&i2c->dev, "%s: uvlo_fd(0x%02X)\n", __func__, uvlo_fd);

	enabled = !(uvlo_fd & 1);

	return sprintf(buf, "%s\n", enabled ? "enabled" :  "disabled");
}

static ssize_t enable_fd_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct s2dos05_data *info = dev_get_drvdata(dev);
	struct i2c_client *i2c = info->iodev->i2c;
	int ret;
	bool enable;
	u8 uvlo_fd;

	ret = strtobool(buf, &enable);
	if (ret)
		return ret;

	dev_info(&i2c->dev, "%s: enable(%d)\n", __func__, enable);

	uvlo_fd = !enable;

	ret = s2dos05_update_reg(i2c, S2DOS05_REG_UVLO_FD, uvlo_fd, 1);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: Failed to update FD(%d)\n", __func__, ret);
		return ret;
	}

	return count;
}

static DEVICE_ATTR(enable_fd, 0664, enable_fd_show, enable_fd_store);

static void __maybe_unused handle_fd_work(struct work_struct *work)
{
	struct s2dos05_data *s2dos05 =
		container_of(to_delayed_work(work), struct s2dos05_data, fd_work);
	struct i2c_client *i2c = s2dos05->iodev->i2c;
	u8 uvlo_fd = 0;
	int ret;

	dev_info(&i2c->dev, "%s: Enable FD\n", __func__);
	ret = s2dos05_update_reg(i2c, S2DOS05_REG_UVLO_FD, 0, 1);
	if (ret < 0)
		dev_err(&i2c->dev, "%s: Failed to enable FD(%d)\n", __func__, ret);
	s2dos05_read_reg(i2c, S2DOS05_REG_UVLO_FD, &uvlo_fd);
	dev_info(&i2c->dev, "%s: uvlo_fd(0x%02X)\n", __func__, uvlo_fd);
}

static int __maybe_unused fb_state_change(struct notifier_block *nb, unsigned long val,
			   void *data)
{
	struct s2dos05_data *s2dos05 =
		container_of(nb, struct s2dos05_data, fb_block);
#if !IS_ENABLED(CONFIG_SEC_PM_QCOM)
	struct i2c_client *i2c = s2dos05->iodev->i2c;
	struct fb_event *evdata = data;
	struct fb_info *info = evdata->info;
	int ret;
#endif
	unsigned int blank;

	if (val != FB_EVENT_BLANK)
		return 0;


#if IS_ENABLED(CONFIG_SEC_PM_QCOM)
	blank = *(int *)data;
#else
	/*
	 * If FBNODE is not zero, it is not primary display(LCD)
	 * and don't need to process these scheduling.
	 */
	if (info->node)
		return NOTIFY_OK;

	blank = *(int *)evdata->data;
#endif

	if (blank == FB_BLANK_UNBLANK) {
#if IS_ENABLED(CONFIG_SEC_PM_QCOM)
		schedule_delayed_work(&s2dos05->fd_work, msecs_to_jiffies(500));
#else
		dev_info(&i2c->dev, "%s: Enable FD\n", __func__);
		ret = s2dos05_update_reg(i2c, S2DOS05_REG_UVLO_FD, 0, 1);
		if (ret < 0)
			dev_err(&i2c->dev, "%s: Failed to enable FD(%d)\n",
					__func__, ret);
#endif
	}
#if IS_ENABLED(CONFIG_SEC_PM_QCOM)
	else {
		cancel_delayed_work_sync(&s2dos05->fd_work);
	}
#endif

	return NOTIFY_OK;
}

#if IS_ENABLED(CONFIG_SEC_FACTORY)
#define VALID_REG	S2DOS05_REG_EN	/* Register address for validation */
#define VALID_MASK	0xE0		/* NA(reserved) bit */

static ssize_t validation_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s2dos05_data *s2dos05 = dev_get_drvdata(dev);
	struct i2c_client *i2c = s2dos05->iodev->i2c;
	int ret;
	bool result = false;
	u8 val;

	ret = s2dos05_read_reg(i2c, VALID_REG, &val);
	if (ret < 0) {
		dev_err(dev, "%s: fail to read reg\n", __func__);
		goto out;
	}

	dev_info(dev, "%s: initial state: reg(0x%02X) data(0x%02X)\n", __func__,
			VALID_REG, val);

	ret = s2dos05_update_reg(i2c, VALID_REG, VALID_MASK, VALID_MASK);
	if (ret < 0) {
		dev_err(dev, "%s: fail to update reg\n", __func__);
		goto out;
	}

	ret = s2dos05_read_reg(i2c, VALID_REG, &val);
	if (ret < 0) {
		dev_err(dev, "%s: fail to read reg\n", __func__);
		goto out;
	}

	dev_info(dev, "%s: updated state: reg(0x%02x) data(0x%02x)\n", __func__,
			VALID_REG, val);

	result = (val & VALID_MASK) == VALID_MASK;

	/* No need change to init value(0x00), but, do it */
	s2dos05_update_reg(i2c, VALID_REG, 0x00, VALID_MASK);

out:
	dev_info(dev, "%s: result: %s\n", __func__, result ? "ok" : "not ok");

	return sprintf(buf, "%d\n", result);
}
static DEVICE_ATTR(validation, 0444, validation_show, NULL);

/* for LDO burnt: enable/disable all regulator, support only for sm3080 due to hidden register */
#define NUM_REG	7	/* number of regulator to control */

struct power_sequence {
    int regulator;
    int delay;		/* delay after regulator control */
};

static ssize_t enable_pwr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct s2dos05_data *info = dev_get_drvdata(dev);
	struct i2c_client *i2c = info->iodev->i2c;
	u8 reg_status, ctrl;
	u8 val, mask;
	int i;
	bool enable;
	int ret;

	/* { regulator - delay } sequence */
	struct power_sequence en_order[NUM_REG] = {
					{ S2DOS05_LDO1, 0 },
					{ S2DOS05_LDO4, 900 },
					{ SM3080_AVDD, 100 },
					{ SM3080_ELVDD, 10 },
					{ SM3080_ELVSS, 1000 },
					{ S2DOS05_LDO2, 0 },
					{ S2DOS05_LDO3, 0 } };
	struct power_sequence dis_order[NUM_REG] = {
					{ S2DOS05_LDO3, 5 },
					{ S2DOS05_LDO2, 450 },
					{ SM3080_ELVDD, 0 },
					{ SM3080_ELVSS, 50 },
					{ SM3080_AVDD, 50 },
					{ S2DOS05_LDO4, 50 },
					{ S2DOS05_LDO1, 10 } };
	struct power_sequence *order;	/* be selected by enable */

	ret = strtobool(buf, &enable);
	if (ret)
		return ret;

	s2dos05_read_reg(i2c, S2DOS05_REG_STAT, &reg_status);
	s2dos05_read_reg(i2c, S2DOS05_REG_EN, &ctrl);
	dev_info(&i2c->dev, "++%s: en(%d), status(0x%02x), ctrl(0x%02x)\n",
					__func__, enable, reg_status, ctrl);

	order = enable ? en_order : dis_order;

	for (i = 0; i < NUM_REG; i++) {
		val = enable << order[i].regulator;
		mask = 1 << order[i].regulator;

		ret = s2dos05_update_reg(i2c, S2DOS05_REG_EN, val, mask);

		msleep(order[i].delay);
	}

	/* enable fd because of el power on */
	if (enable)
		ret = s2dos05_update_reg(i2c, S2DOS05_REG_UVLO_FD, 0, 1);

	s2dos05_read_reg(i2c, S2DOS05_REG_STAT, &reg_status);
	s2dos05_read_reg(i2c, S2DOS05_REG_EN, &ctrl);
	dev_info(&i2c->dev, "--%s: en(%d), status(0x%02x), ctrl(0x%02x)\n",
					__func__, enable, reg_status, ctrl);

	return count;
}

static ssize_t enable_pwr_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct s2dos05_data *s2dos05 = dev_get_drvdata(dev);
    struct i2c_client *i2c = s2dos05->iodev->i2c;
    u8 reg_status, ctrl;

    s2dos05_read_reg(i2c, S2DOS05_REG_STAT, &reg_status);
    s2dos05_read_reg(i2c, S2DOS05_REG_EN, &ctrl);
    dev_info(&i2c->dev, "%s: status(0x%02x), ctrl(0x%02x)\n",
                    __func__, reg_status, ctrl);

    return sprintf(buf, "0x%02x\n0x%02x\n", reg_status, ctrl);
}
static DEVICE_ATTR(enable_pwr, 0664, enable_pwr_show, enable_pwr_store);
#endif /* CONFIG_SEC_FACTORY */

#if IS_ENABLED(CONFIG_SEC_PM) && IS_ENABLED(CONFIG_SEC_PM_QCOM)
static void sec_set_fd(struct s2dos05_data *info)
{
	info->fb_block.notifier_call = fb_state_change;
	msm_drm_register_notifier_client(&info->fb_block);
	INIT_DELAYED_WORK(&info->fd_work, handle_fd_work);
	info->fd_work_init = false;
}
#elif IS_ENABLED(CONFIG_SEC_PM) && IS_ENABLED(CONFIG_REGULATOR_S2DOS05_ELVSS_FD)
static void sec_set_fd(struct s2dos05_data *info)
{
	INIT_DELAYED_WORK(&info->fd_work, handle_fd_work);
	info->fd_work_init = true;
}
#elif IS_ENABLED(CONFIG_SEC_PM)
static void sec_set_fd(struct s2dos05_data *info)
{
	info->fb_block.notifier_call = fb_state_change;
	fb_register_client(&info->fb_block);
	info->fd_work_init = false;
}
#else
static void sec_set_fd(struct s2dos05_data *info)
{
	info->fd_work_init = false;
}
#endif

static int s2dos05_sec_pm_init(struct s2dos05_data *info)
{
	struct s2dos05_dev *iodev = info->iodev;
	struct device *dev = &iodev->i2c->dev;
	const char *sec_disp_pmic_name = iodev->pdata->sec_disp_pmic_name;
	int ret = 0;

	if (sec_disp_pmic_name)
		iodev->sec_disp_pmic_dev =
			sec_device_create(info, sec_disp_pmic_name);
	else
		iodev->sec_disp_pmic_dev =
			sec_device_create(info, "disp_pmic");

	if (unlikely(IS_ERR(iodev->sec_disp_pmic_dev))) {
		ret = PTR_ERR(iodev->sec_disp_pmic_dev);
		dev_err(dev, "%s: Failed to create disp_pmic(%d)\n", __func__,
				ret);
		return ret;
	}

	dev_info(dev, "%s: Enable FD\n", __func__);
	ret = s2dos05_update_reg(iodev->i2c, S2DOS05_REG_UVLO_FD, 0, 1);
	if (ret < 0) {
		dev_err(dev, "%s: Failed to enable FD(%d)\n", __func__, ret);
		goto remove_sec_disp_pmic_dev;
	}

	/* To separate FD operation */
	sec_set_fd(info);

	ret = device_create_file(iodev->sec_disp_pmic_dev, &dev_attr_enable_fd);
	if (ret) {
		dev_err(dev, "%s: Failed to create enable_fd(%d)\n", __func__,
				ret);
		goto remove_sec_disp_pmic_dev;
	}

#if IS_ENABLED(CONFIG_SEC_FACTORY)
	ret = device_create_file(iodev->sec_disp_pmic_dev, &dev_attr_validation);
	if (ret) {
		pr_err("s2dos05_sysfs: failed to create validation file, %s\n",
			dev_attr_validation.attr.name);
		goto remove_sec_disp_enable_fd;
	}

	if (iodev->is_sm3080) {
		ret = device_create_file(iodev->sec_disp_pmic_dev, &dev_attr_enable_pwr);
		if (ret) {
			pr_err("s2dos05_sysfs: failed to create enable_pwr file, %s\n",
				dev_attr_enable_pwr.attr.name);
			goto remove_sec_disp_validation;
		}
	}
#endif /* CONFIG_SEC_FACTORY */

	return 0;

#if IS_ENABLED(CONFIG_SEC_FACTORY)
remove_sec_disp_validation:
	device_remove_file(info->iodev->sec_disp_pmic_dev, &dev_attr_validation);
remove_sec_disp_enable_fd:
	device_remove_file(info->iodev->sec_disp_pmic_dev, &dev_attr_enable_fd);
#endif /* CONFIG_SEC_FACTORY */
remove_sec_disp_pmic_dev:
	sec_device_destroy(iodev->sec_disp_pmic_dev->devt);

	return ret;
}

static void s2dos05_sec_pm_deinit(struct s2dos05_data *info)
{
	device_remove_file(info->iodev->sec_disp_pmic_dev, &dev_attr_enable_fd);
#if IS_ENABLED(CONFIG_SEC_PM_QCOM)
	msm_drm_unregister_notifier_client(&info->fb_block);
#endif
#if IS_ENABLED(CONFIG_SEC_FACTORY)
	device_remove_file(info->iodev->sec_disp_pmic_dev, &dev_attr_validation);
	device_remove_file(info->iodev->sec_disp_pmic_dev, &dev_attr_enable_pwr);
#endif /* CONFIG_SEC_FACTORY */
	sec_device_destroy(info->iodev->sec_disp_pmic_dev->devt);
}
#endif /* CONFIG_SEC_PM */

static int s2dos05_pmic_probe(struct i2c_client *i2c,
				const struct i2c_device_id *dev_id)
{
	struct s2dos05_dev *iodev;
	struct s2dos05_platform_data *pdata = i2c->dev.platform_data;
	struct regulator_config config = { };
	struct s2dos05_data *s2dos05;
	size_t i;
	int ret = 0;
	u8 val = 0, mask = 0;

	pr_info("%s:%s\n", MFD_DEV_NAME, __func__);

	iodev = devm_kzalloc(&i2c->dev, sizeof(struct s2dos05_dev), GFP_KERNEL);
	if (!iodev) {
		dev_err(&i2c->dev, "%s: Failed to alloc mem for s2dos05\n",
							__func__);
		return -ENOMEM;
	}

	if (i2c->dev.of_node) {
		pdata = devm_kzalloc(&i2c->dev,
			sizeof(struct s2dos05_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&i2c->dev, "Failed to allocate memory\n");
			ret = -ENOMEM;
			goto err_pdata;
		}

		ret = s2dos05_pmic_dt_parse_pdata(&i2c->dev, pdata);
		if (ret < 0) {
			dev_err(&i2c->dev, "Failed to get device of_node\n");
			goto err_pdata;
		}

		i2c->dev.platform_data = pdata;
	} else
		pdata = i2c->dev.platform_data;

	iodev->dev = &i2c->dev;
	iodev->i2c = i2c;

	if (pdata) {
		iodev->pdata = pdata;
		iodev->wakeup = pdata->wakeup;
	} else {
		ret = -EINVAL;
		goto err_pdata;
	}
	mutex_init(&iodev->i2c_lock);

	s2dos05 = devm_kzalloc(&i2c->dev, sizeof(struct s2dos05_data),
				GFP_KERNEL);
	if (!s2dos05) {
		pr_info("[%s:%d] if (!s2dos05)\n", __FILE__, __LINE__);
		ret = -ENOMEM;
		goto err_s2dos05_data;
	}

#if IS_ENABLED(CONFIG_SEC_ABC)
	atomic_set(&s2dos05->i2c_fail_count, 0);
#endif /* CONFIG_SEC_ABC */
	i2c_set_clientdata(i2c, s2dos05);
	s2dos05->iodev = iodev;
	s2dos05->num_regulators = pdata->num_rdata;

	for (i = 0; i < pdata->num_rdata; i++) {
		int id = pdata->regulators[i].id;
		config.dev = &i2c->dev;
		config.init_data = pdata->regulators[i].initdata;
		config.driver_data = s2dos05;
		config.of_node = pdata->regulators[i].reg_node;
		s2dos05->rdev[i] = devm_regulator_register(&i2c->dev,
							   &regulators[id], &config);
		if (IS_ERR(s2dos05->rdev[i])) {
			ret = PTR_ERR(s2dos05->rdev[i]);
			dev_err(&i2c->dev, "regulator init failed for %d\n",
				id);
			s2dos05->rdev[i] = NULL;
			goto err_s2dos05_data;
		}
#if IS_ENABLED(CONFIG_REGULATOR_DEBUG_CONTROL)
		ret = devm_regulator_debug_register(&i2c->dev, s2dos05->rdev[i]);
		if (ret)
			dev_err(&i2c->dev, "failed to register debug regulator for %d, rc=%d\n",
					i, ret);
#endif
	}

#if IS_ENABLED(CONFIG_SEC_PM)
	ret = s2dos05_read_reg(i2c, S2DOS05_REG_DEVICE_ID_PGM, &val);
	if (ret < 0) {
		dev_err(&i2c->dev, "Failed to read DEVICE ID address\n");
		goto err_s2dos05_data;
	}

	if (val & (1 << 7)) {
		iodev->is_sm3080 = true;
		dev_info(&i2c->dev, "SM3080 DEVICE ID: 0x%02X\n", val);
	}

	/* set OCL_ELVSS */
	if (pdata->ocl_elvss > -1) {
		s2dos05_update_reg(i2c, S2DOS05_REG_OCL, pdata->ocl_elvss, S2DOS05_OCL_ELVSS_MASK);
		pr_info("%s: set ocl elvss: %d\n", __func__, pdata->ocl_elvss);
	}

	ret = s2dos05_sec_pm_init(s2dos05);
	if (ret < 0)
		goto err_s2dos05_data;
#endif /* CONFIG_SEC_PM */

	iodev->adc_mode = pdata->adc_mode;
	iodev->adc_sync_mode = pdata->adc_sync_mode;
	if (iodev->adc_mode > 0)
		s2dos05_powermeter_init(iodev);

	val = (S2DOS05_IRQ_PWRMT_MASK | S2DOS05_IRQ_TSD_MASK
		| S2DOS05_IRQ_UVLO_MASK | S2DOS05_IRQ_OCD_MASK);
	mask = (S2DOS05_IRQ_PWRMT_MASK | S2DOS05_IRQ_TSD_MASK | S2DOS05_IRQ_SSD_MASK
		| S2DOS05_IRQ_SCP_MASK | S2DOS05_IRQ_UVLO_MASK | S2DOS05_IRQ_OCD_MASK);
	ret = s2dos05_update_reg(iodev->i2c, S2DOS05_REG_IRQ_MASK, val, mask);
	if (ret < 0) {
		dev_err(&i2c->dev, "Failed to mask IRQ MASK address\n");
		return ret;
	}

	if (pdata->dp_pmic_irq > 0) {
		iodev->dp_pmic_irq = gpio_to_irq(pdata->dp_pmic_irq);
		pr_info("%s : dp_pmic_irq = %d\n", __func__, iodev->dp_pmic_irq);
		if (iodev->dp_pmic_irq > 0) {
			ret = request_threaded_irq(iodev->dp_pmic_irq,
					NULL, s2dos05_irq_thread,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"dp-pmic-irq", s2dos05);
			if (ret) {
				dev_err(&i2c->dev,
						"%s: Failed to Request IRQ\n", __func__);
				goto err_s2dos05_data;
			}

			ret = enable_irq_wake(iodev->dp_pmic_irq);
			if (ret < 0)
				dev_err(&i2c->dev,
						"%s: Failed to Enable Wakeup Source(%d)\n",
						__func__, ret);
		} else {
			dev_err(&i2c->dev, "%s: Failed gpio_to_irq(%d)\n",
					__func__, iodev->dp_pmic_irq);
			goto err_s2dos05_data;
		}
	}
#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
	ret = s2dos05_create_sysfs(s2dos05);
	if (ret < 0) {
		pr_err("%s: s2dos05_create_sysfs fail\n", __func__);
		goto err_s2dos05_data;
	}
#endif
	return ret;

err_s2dos05_data:
	mutex_destroy(&iodev->i2c_lock);
err_pdata:
	return ret;
}

#if IS_ENABLED(CONFIG_OF)
static struct of_device_id s2dos05_i2c_dt_ids[] = {
	{ .compatible = "samsung,s2dos05pmic" },
	{ },
};
#endif /* CONFIG_OF */

static int __s2dos05_pmic_remove(struct i2c_client *i2c)
{
	struct s2dos05_data *info = i2c_get_clientdata(i2c);
#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
	struct device *s2dos05_pmic = info->dev;
	int i = 0;

	dev_info(&i2c->dev, "%s\n", __func__);

	/* Remove sysfs entries */
	for (i = 0; i < ATTR_REGULATOR; i++)
		device_remove_file(s2dos05_pmic, &regulator_attr[i].dev_attr);
	pmic_device_destroy(s2dos05_pmic->devt);
#else
	dev_info(&i2c->dev, "%s\n", __func__);
#endif
	if (info->iodev->adc_mode > 0)
		s2dos05_powermeter_deinit(info->iodev);

#if IS_ENABLED(CONFIG_SEC_PM)
	s2dos05_sec_pm_deinit(info);
#endif /* CONFIG_SEC_PM */

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
static void s2dos05_pmic_remove(struct i2c_client *i2c)
{
	__s2dos05_pmic_remove(i2c);
}
#else
static int s2dos05_pmic_remove(struct i2c_client *i2c)
{
	return __s2dos05_pmic_remove(i2c);
}
#endif

#if IS_ENABLED(CONFIG_PM)
static int s2dos05_pmic_suspend(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct s2dos05_data *info = i2c_get_clientdata(i2c);
	struct s2dos05_dev *s2dos05 = info->iodev;

	pr_info("%s adc_mode : %d\n", __func__, s2dos05->adc_mode);
	if (s2dos05->adc_mode > 0) {
		s2dos05_read_reg(s2dos05->i2c,
				 S2DOS05_REG_PWRMT_CTRL2, &s2dos05->adc_en_val);
		if (s2dos05->adc_en_val & 0x80)
			s2dos05_update_reg(s2dos05->i2c,
					   S2DOS05_REG_PWRMT_CTRL2,
					   0, ADC_EN_MASK);
	}

	return 0;
}

static int s2dos05_pmic_resume(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct s2dos05_data *info = i2c_get_clientdata(i2c);
	struct s2dos05_dev *s2dos05 = info->iodev;

	pr_info("%s adc_mode : %d\n", __func__, s2dos05->adc_mode);
	if (s2dos05->adc_mode > 0) {
#if IS_ENABLED(CONFIG_SEC_PM)
		int ret;
		ret = s2dos05_update_reg(s2dos05->i2c, S2DOS05_REG_PWRMT_CTRL2,
				s2dos05->adc_en_val & 0x80, ADC_EN_MASK);
		if (ret < 0)
			pr_err("%s: Failed to update_reg: %d\n", __func__, ret);
#else
		s2dos05_update_reg(s2dos05->i2c, S2DOS05_REG_PWRMT_CTRL2,
				   s2dos05->adc_en_val & 0x80, ADC_EN_MASK);
#endif /* CONFIG_SEC_PM */
	}

	return 0;
}
#else
#define s2dos05_pmic_suspend	NULL
#define s2dos05_pmic_resume	NULL
#endif /* CONFIG_PM */

static const struct dev_pm_ops s2dos05_pmic_pm = {
	.suspend = s2dos05_pmic_suspend,
	.resume = s2dos05_pmic_resume,
};

#if IS_ENABLED(CONFIG_OF)
static const struct i2c_device_id s2dos05_pmic_id[] = {
	{"s2dos05-regulator", 0},
	{},
};
#endif

static struct i2c_driver s2dos05_i2c_driver = {
	.driver = {
		.name = "s2dos05-regulator",
		.owner = THIS_MODULE,
		.pm = &s2dos05_pmic_pm,
#if IS_ENABLED(CONFIG_OF)
		.of_match_table	= s2dos05_i2c_dt_ids,
#endif /* CONFIG_OF */
		.suppress_bind_attrs = true,
	},
	.probe = s2dos05_pmic_probe,
	.remove = s2dos05_pmic_remove,
	.id_table = s2dos05_pmic_id,
};

static int __init s2dos05_i2c_init(void)
{
	pr_info("%s:%s\n", MFD_DEV_NAME, __func__);
	return i2c_add_driver(&s2dos05_i2c_driver);
}
subsys_initcall(s2dos05_i2c_init);

static void __exit s2dos05_i2c_exit(void)
{
	i2c_del_driver(&s2dos05_i2c_driver);
}
module_exit(s2dos05_i2c_exit);

MODULE_DESCRIPTION("SAMSUNG s2dos05 Regulator Driver");
MODULE_LICENSE("GPL");
